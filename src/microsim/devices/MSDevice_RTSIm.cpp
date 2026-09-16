/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2026 RTSIm contributors.
// SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later
/****************************************************************************/
#include <config.h>
#include <cmath>
#include <cstdint>
#include <map>
#include <limits>
#include <memory>
#include <algorithm>
#include <set>
#include <sstream>
#include <utils/common/StringUtils.h>
#include <utils/common/RandHelper.h>
#include <utils/common/WrappingCommand.h>
#include <utils/options/OptionsCont.h>
#include <utils/iodevices/OutputDevice.h>
#include <utils/emissions/EnergyParams.h>
#include <utils/emissions/PollutantsInterface.h>
#include <microsim/MSVehicle.h>
#include <microsim/MSNet.h>
#include <microsim/MSEventControl.h>
#include <microsim/MSVehicleControl.h>
#include <microsim/MSLane.h>
#include <microsim/MSEdge.h>
#include <microsim/cfmodels/CC_Const.h>
#include <microsim/cfmodels/ParBuffer.h>
#include "MSDevice_RTSIm.h"
#include "RTSImCFDTable.h"

namespace {
std::map<std::string, std::shared_ptr<RTSIm::CFDTable> > tables;
// Immutable scenario identities, retained after departures/arrivals. No vehicle
// pointers are stored here; cleanup resets the registry between simulations.
std::map<std::string, std::vector<std::string> > platoons;
std::map<std::string, std::string> memberPlatoons;

std::string joinedMembers(const std::vector<std::string>& members) {
    std::string result;
    for (const std::string& member : members) {
        if (!result.empty()) { result += " "; }
        result += member;
    }
    return result;
}

std::string parameter(const SUMOVehicle& v, const std::string& key, const std::string& def = "") {
    const std::string name = "device.rtsim." + key;
    return v.getParameter().getParameter(name, v.getVehicleType().getParameter().getParameter(name, def));
}

double number(const std::string& value, const std::string& name) {
    double n;
    try { n = StringUtils::toDouble(value); }
    catch (...) { throw InvalidArgument("RTSIm: invalid numeric value for " + name + ": " + value); }
    if (!std::isfinite(n)) { throw InvalidArgument("RTSIm: non-finite " + name); }
    return n;
}

int integer(const std::string& value, const std::string& name) {
    try { return StringUtils::toInt(value); }
    catch (...) { throw InvalidArgument("RTSIm: invalid integer for " + name + ": " + value); }
}

MSVehicle* cooperativePeer(const std::string& id) {
    SUMOVehicle* peer = MSNet::getInstance()->getVehicleControl().getVehicle(id);
    if (peer == nullptr) { return nullptr; }
    MSVehicle* micro = dynamic_cast<MSVehicle*>(peer);
    // Plexe auto feeding casts peers' controller variables to CC variables.
    // Check the actual model before any peer is handed to that API.
    if (micro == nullptr || micro->getCarFollowModel().getModelID() != SUMO_TAG_CF_CC) {
        throw InvalidArgument("RTSIm cooperative peer '" + id + "' must use carFollowModel=CC");
    }
    return micro;
}

void validatePeerID(const std::string& id, const std::string& field, const std::string& self) {
    if (id.empty() || id == self || id.find_first_of(" \t\r\n\f\v") != std::string::npos) {
        throw InvalidArgument("RTSIm cooperative " + field + " requires a nonempty vehicle ID without whitespace, different from this vehicle");
    }
}

double surface(const std::string& road) {
    if (road == "primary") { return 0.006923; }
    if (road == "secondary") { return 0.010; }
    if (road == "cross_country") { return 0.025; }
    throw InvalidArgument("RTSIm: unknown road-type '" + road + "'");
}
}

void MSDevice_RTSIm::insertOptions(OptionsCont& oc) {
    oc.addOptionSubTopic("RTSIm Device");
    insertDefaultAssignmentOptions("rtsim", "RTSIm Device", oc);
    oc.doRegister("device.rtsim.cfd-file", new Option_FileName());
    oc.addDescription("device.rtsim.cfd-file", "RTSIm Device", "CFD coefficient CSV for explicit RTSIm configurations");
}

void MSDevice_RTSIm::buildVehicleDevices(SUMOVehicle& v, std::vector<MSVehicleDevice*>& into) {
    if (equippedByDefaultAssignmentOptions(OptionsCont::getOptions(), "rtsim", v, false)) {
        into.push_back(new MSDevice_RTSIm(v));
    }
}

void MSDevice_RTSIm::cleanup() {
    tables.clear();
    platoons.clear();
    memberPlatoons.clear();
}

MSDevice_RTSIm::MSDevice_RTSIm(SUMOVehicle& holder) : MSVehicleDevice(holder, "rtsim_" + holder.getID()) {
    MSVehicle* micro = dynamic_cast<MSVehicle*>(&holder);
    if (micro == nullptr) { throw InvalidArgument("RTSIm currently requires microscopic simulation"); }
    if (OptionsCont::getOptions().isSet("load-state")) { throw InvalidArgument("RTSIm state restoration is not implemented"); }
    if (OptionsCont::getOptions().getInt("threads") > 1) { throw InvalidArgument("RTSIm prototype requires a single simulation thread"); }
    myLevel = integer(parameter(holder, "automation-level", "-1"), "automation-level");
    if (myLevel < -1 || myLevel > 5) { throw InvalidArgument("RTSIm automation-level must be 0..5 or -1 (unchanged)"); }
    // Research presets from RTSim/data/automation_levels.csv; not SAE capability claims.
    static const double sigma[] = {0.5, 0.4, 0.3, 0.2, 0., 0.};
    static const double tau[] = {1., .95, .90, .80, .70, .60};
    if (myLevel >= 0) { mySigma = sigma[myLevel]; myTau = tau[myLevel]; }
    const std::string sigmaText = parameter(holder, "sigma");
    const std::string tauText = parameter(holder, "tau");
    if (!sigmaText.empty()) { mySigma = number(sigmaText, "sigma"); if (mySigma < 0.) { throw InvalidArgument("RTSIm explicit sigma must be nonnegative"); } }
    if (!tauText.empty()) { myTau = number(tauText, "tau"); if (myTau <= 0.) { throw InvalidArgument("RTSIm explicit tau must be positive"); } }
    if ((mySigma != -1. && (mySigma < 0. || mySigma > 1.)) || (myTau != -1. && myTau <= 0.)) {
        throw InvalidArgument("RTSIm requires sigma in [0,1] and tau > 0");
    }
    const int cf = micro->getCarFollowModel().getModelID();
    myRequestedController = parameter(holder, "controller", "unchanged");
    const std::string& controller = myRequestedController;
    if (controller != "unchanged" && controller != "ACC" && controller != "CACC" && controller != "PLOEG") {
        throw InvalidArgument("RTSIm controller must be ACC, CACC, PLOEG or unchanged");
    }
    if (controller == "ACC" && cf != SUMO_TAG_CF_CC && cf != SUMO_TAG_CF_ACC) { throw InvalidArgument("RTSIm ACC requires carFollowModel=CC or ACC"); }
    if (controller == "CACC" && cf != SUMO_TAG_CF_CC && cf != SUMO_TAG_CF_CACC) { throw InvalidArgument("RTSIm CACC requires carFollowModel=CC or CACC"); }
    if (controller == "PLOEG" && cf != SUMO_TAG_CF_CC) { throw InvalidArgument("RTSIm PLOEG requires carFollowModel=CC"); }
    myPlexe = cf == SUMO_TAG_CF_CC && controller != "unchanged";
    myCooperative = myPlexe && (controller == "CACC" || controller == "PLOEG");
    if ((mySigma >= 0. || myTau > 0.) && cf != SUMO_TAG_CF_KRAUSS && cf != SUMO_TAG_CF_ACC && cf != SUMO_TAG_CF_CACC && !myPlexe) {
        throw InvalidArgument("RTSIm automation requires Krauss, ACC, CACC or explicitly enabled CC/ACC, CC/CACC or CC/PLOEG");
    }
    if (myPlexe) {
        myDesiredSpeed = number(parameter(holder, "desired-speed", "-1"), "desired-speed");
        if (myDesiredSpeed < 0.) { throw InvalidArgument("RTSIm CC/" + controller + " requires desired-speed >= 0"); }
        myRequestedPlexeController = controller == "PLOEG" ? Plexe::PLOEG : (controller == "CACC" ? Plexe::CACC : Plexe::ACC);
    }
    myLeaderID = parameter(holder, "leader");
    myFrontID = parameter(holder, "front");
    const std::string sizeText = parameter(holder, "platoon-size");
    const std::string positionText = parameter(holder, "position");
    mySize = integer(sizeText.empty() ? "1" : sizeText, "platoon-size");
    myPosition = integer(positionText.empty() ? "1" : positionText, "position");
    myPlatoonID = parameter(holder, "platoon-id");
    const std::string membersText = parameter(holder, "members");
    if (myPlatoonID.empty() != membersText.empty()) {
        throw InvalidArgument("RTSIm platoon-id and members must be specified together");
    }
    if (!myPlatoonID.empty()) {
        if (myPlatoonID.find_first_of(" \t\r\n\f\v") != std::string::npos) {
            throw InvalidArgument("RTSIm platoon-id must not contain whitespace");
        }
        std::istringstream members(membersText);
        std::string member;
        std::set<std::string> unique;
        while (members >> member) {
            if (!unique.insert(member).second) {
                throw InvalidArgument("RTSIm members contains duplicate vehicle '" + member + "'");
            }
            myMembers.push_back(member);
        }
        const auto self = std::find(myMembers.begin(), myMembers.end(), holder.getID());
        if (self == myMembers.end()) { throw InvalidArgument("RTSIm members must include this vehicle"); }
        const int size = (int)myMembers.size();
        const int position = (int)std::distance(myMembers.begin(), self) + 1;
        const std::string leader = myMembers.front();
        const std::string front = position == 1 ? "" : myMembers[position - 2];
        if ((!sizeText.empty() && mySize != size) || (!positionText.empty() && myPosition != position)
                || (!myLeaderID.empty() && myLeaderID != leader) || (!myFrontID.empty() && myFrontID != front)) {
            throw InvalidArgument("RTSIm leader/front/position/platoon-size conflicts with ordered members");
        }
        mySize = size;
        myPosition = position;
        myLeaderID = leader;
        myFrontID = front;
        const auto group = platoons.find(myPlatoonID);
        if (group != platoons.end() && group->second != myMembers) {
            throw InvalidArgument("RTSIm members must be identical and ordered consistently for platoon '" + myPlatoonID + "'");
        }
        for (const std::string& id : myMembers) {
            const auto assigned = memberPlatoons.find(id);
            if (assigned != memberPlatoons.end() && assigned->second != myPlatoonID) {
                throw InvalidArgument("RTSIm vehicle '" + id + "' is already assigned to platoon '" + assigned->second + "'");
            }
        }
        // The designated leader has no upstream platoon data. Its identity is
        // still retained if a cooperative controller was requested for all members.
        if (position == 1) { myCooperative = false; }
    }
    if (myCooperative) {
        validatePeerID(myLeaderID, "leader", holder.getID());
        validatePeerID(myFrontID, "front", holder.getID());
        ParBuffer encoded;
        encoded << 1 << myLeaderID << myFrontID;
        ParBuffer decoded(encoded.str());
        int enabled = 0;
        std::string leaderID, frontID;
        decoded >> enabled >> leaderID >> frontID;
        if (enabled != 1 || leaderID != myLeaderID || frontID != myFrontID) {
            throw InvalidArgument("RTSIm leader/front IDs cannot be represented by Plexe's parameter encoding");
        }
        // Peers may be loaded or inserted later; known incompatible peers fail now.
        cooperativePeer(myLeaderID);
        cooperativePeer(myFrontID);
    } else if (myPlatoonID.empty() && (!myLeaderID.empty() || !myFrontID.empty())) {
        throw InvalidArgument("RTSIm leader/front require carFollowModel=CC and controller=CACC or PLOEG");
    }
    const std::string spacingText = parameter(holder, "cacc-spacing");
    if (myPlexe && controller == "CACC") {
        if (spacingText.empty()) { throw InvalidArgument("RTSIm CC/CACC requires explicit cacc-spacing > 0"); }
        myCACCSpacing = number(spacingText, "cacc-spacing");
        if (myCACCSpacing <= 0.) { throw InvalidArgument("RTSIm cacc-spacing must be positive"); }
    } else if (!spacingText.empty()) {
        throw InvalidArgument("RTSIm cacc-spacing requires carFollowModel=CC and controller=CACC");
    }
    const std::string sigmaStepText = parameter(holder, "sigma-step");
    if (cf == SUMO_TAG_CF_KRAUSS && !sigmaStepText.empty()) {
        throw InvalidArgument("RTSIm device.rtsim.sigma-step is unsupported for Krauss; use the native vType sigmaStep attribute");
    }
    const double sigmaStepSeconds = number(sigmaStepText.empty() ? toString(TS, 17) : sigmaStepText, "sigma-step");
    if (sigmaStepSeconds <= 0.) { throw InvalidArgument("RTSIm sigma-step must be positive"); }
    const bool configureSigmaStep = !sigmaStepText.empty() || (mySigma >= 0. && (cf == SUMO_TAG_CF_ACC || cf == SUMO_TAG_CF_CACC || myPlexe));
    if (configureSigmaStep) {
        checkTimeBounds(sigmaStepSeconds);
        mySigmaStep = TIME2STEPS(sigmaStepSeconds);
        if (mySigmaStep < DELTA_T || mySigmaStep % DELTA_T != 0 || std::abs(STEPS2TIME(mySigmaStep) - sigmaStepSeconds) > 1e-9) {
            throw InvalidArgument("RTSIm sigma-step must be an exact multiple of the simulation timestep");
        }
    }
    if (mySigma > 0.) { myNoiseRNG.setSeed((static_cast<uint32_t>(holder.getRandomSeed()) ^ 0x52745349u) & 0x7fffffffu); }

    myRoadType = parameter(holder, "road-type");
    if (!myRoadType.empty()) { myDefaultFr0 = surface(myRoadType); }
    const std::string fr0 = parameter(holder, "fr0");
    if (!fr0.empty()) {
        myDefaultFr0 = number(fr0, "fr0");
        if (myDefaultFr0 < 0.) { throw InvalidArgument("RTSIm fr0 must be nonnegative"); }
    }
    myFr0 = myDefaultFr0;
    const std::string cd = parameter(holder, "cd");
    if (!cd.empty()) {
        myCd = number(cd, "cd");
        if (myCd <= 0.) { throw InvalidArgument("RTSIm cd must be positive"); }
    }
    myModel = parameter(holder, "model");
    myBound = parameter(holder, "cd-bound", "lower");
    if (myBound != "lower" && myBound != "upper") { throw InvalidArgument("RTSIm cd-bound must be lower or upper"); }
    myGap = number(parameter(holder, "gap", "0"), "gap");
    if (mySize < 1 || myPosition < 1 || myPosition > mySize || myGap < 0.) { throw InvalidArgument("RTSIm invalid platoon size, position, or gap"); }
    if (!myModel.empty()) {
        if (!cd.empty()) { throw InvalidArgument("RTSIm specify cd or a CFD model, not both"); }
        OptionsCont& oc = OptionsCont::getOptions();
        if (!oc.isSet("device.rtsim.cfd-file")) { throw InvalidArgument("RTSIm model requires --device.rtsim.cfd-file"); }
        const std::string path = oc.getString("device.rtsim.cfd-file");
        try {
            if (tables.count(path) == 0) {
                std::shared_ptr<RTSIm::CFDTable> table(new RTSIm::CFDTable());
                table->load(path); tables[path] = table;
            }
            myCd = tables.at(path)->lookup(myModel, mySize, myPosition, myGap, myBound);
        } catch (const std::exception& e) { throw InvalidArgument("RTSIm CFD: " + std::string(e.what())); }
    }
    const std::string emission = PollutantsInterface::getName(holder.getVehicleType().getEmissionClass());
    if (emission.find("PHEMlight/") != 0 && emission.find("PHEMlight5/") != 0) {
        throw InvalidArgument("RTSIm prototype requires a PHEMlight or PHEMlight5 emission class");
    }
#ifdef INTERNAL_PHEM
    if (emission.find("PHEMlight/") == 0) { throw InvalidArgument("RTSIm legacy overrides are unsupported with INTERNAL_PHEM"); }
#endif
    // Validate everything before changing this vehicle's state.
    if (!myPlexe && (mySigma >= 0. || myTau > 0.)) {
        MSVehicleType& type = micro->getSingularType();
        if (mySigma >= 0. && cf == SUMO_TAG_CF_KRAUSS) { type.setImperfection(mySigma); }
        if (myTau > 0.) { type.setTau(myTau); }
        type.check();
    }
    if (myPlexe) {
        micro->getCarFollowModel().setParameter(micro, PAR_ACTIVE_CONTROLLER, toString((int)Plexe::ACC));
        // Tau keeps each Plexe controller's native meaning. PATH CACC uses
        // constant spacing; tau affects only its ACC fallback.
        if (myTau > 0.) {
            micro->getCarFollowModel().setParameter(micro, PAR_ACC_HEADWAY_TIME, toString(myTau, 17));
            if (myRequestedPlexeController == Plexe::PLOEG) {
                micro->getCarFollowModel().setParameter(micro, CC_PAR_PLOEG_H, toString(myTau, 17));
            }
        }
        if (myCACCSpacing > 0.) { micro->getCarFollowModel().setParameter(micro, PAR_CACC_SPACING, toString(myCACCSpacing, 17)); }
        if (myCooperative) { micro->getCarFollowModel().setParameter(micro, PAR_USE_AUTO_FEEDING, "0"); }
        applyPlexeDesiredSpeed(myDesiredSpeed);
        if (configureSigmaStep) { micro->getCarFollowModel().setParameter(micro, "rtsim.sigmaStep", toString(sigmaStepSeconds, 17)); }
        if (mySigma >= 0.) { micro->getCarFollowModel().setParameter(micro, "rtsim.sigma", toString(mySigma, 17)); }
    }
    if (myCd > 0.) { holder.getEmissionParameters()->setAirDragCoefficient(myCd); }
    if (myFr0 >= 0.) { holder.getEmissionParameters()->setRollDragCoefficient(myFr0); }
    // Schedule last: construction/parameter exceptions cannot leave a callback
    // pointing at a device whose constructor did not finish.
    const bool newPlatoon = !myPlatoonID.empty() && platoons.count(myPlatoonID) == 0;
    if (newPlatoon) {
        platoons[myPlatoonID] = myMembers;
        for (const std::string& id : myMembers) { memberPlatoons[id] = myPlatoonID; }
    }
    try {
        if (myCooperative || !myPlatoonID.empty()) {
            std::unique_ptr<WrappingCommand<MSDevice_RTSIm> > command(
                new WrappingCommand<MSDevice_RTSIm>(this, &MSDevice_RTSIm::updateCooperativeTopology));
            MSNet::getInstance()->getBeginOfTimestepEvents()->addEvent(command.get(), SIMSTEP + DELTA_T);
            myTopologyCommand = command.release();
        }
    } catch (...) {
        if (newPlatoon) {
            platoons.erase(myPlatoonID);
            for (const std::string& id : myMembers) { memberPlatoons.erase(id); }
        }
        throw;
    }
}

MSDevice_RTSIm::~MSDevice_RTSIm() {
    if (myTopologyCommand != nullptr) { myTopologyCommand->deschedule(); }
}

SUMOTime MSDevice_RTSIm::updateCooperativeTopology(SUMOTime) {
    try {
        MSVehicle* veh = static_cast<MSVehicle*>(&myHolder);
        myFormationIntact = isPlatoonFormationIntact();
        if (!myCooperative) { return DELTA_T; }
        auto& model = veh->getCarFollowModel();
        if (!veh->isOnRoad()) {
            model.setParameter(veh, PAR_USE_AUTO_FEEDING, "0");
            model.setParameter(veh, PAR_ACTIVE_CONTROLLER, toString((int)Plexe::ACC));
            return DELTA_T;
        }
        if (myFormationIntact) {
            // Ideal native state feeding, without a radio/delay model. Refresh
            // IDs each step so no pointer to a removed/replaced peer is reused.
            ParBuffer feed;
            feed << 1 << myLeaderID << myFrontID;
            model.setParameter(veh, PAR_USE_AUTO_FEEDING, feed.str());
            model.setParameter(veh, PAR_ACTIVE_CONTROLLER, toString(myRequestedPlexeController));
            ++myCooperativeSteps;
        } else {
            model.setParameter(veh, PAR_USE_AUTO_FEEDING, "0");
            model.setParameter(veh, PAR_ACTIVE_CONTROLLER, toString((int)Plexe::ACC));
            ++myFallbackSteps;
        }
        return DELTA_T;
    } catch (...) {
        // MSEventControl deletes commands that throw. Do not retain a pointer
        // that the device destructor could subsequently dereference.
        myTopologyCommand = nullptr;
        throw;
    }
}

bool MSDevice_RTSIm::isPlatoonFormationIntact() const {
    if (!myPlatoonID.empty()) {
        MSVehicle* previous = nullptr;
        bool intact = true;
        for (const std::string& id : myMembers) {
            SUMOVehicle* candidate = MSNet::getInstance()->getVehicleControl().getVehicle(id);
            if (candidate == nullptr) { intact = false; previous = nullptr; continue; }
            const MSDevice_RTSIm* device = nullptr;
            for (MSVehicleDevice* item : candidate->getDevices()) {
                device = dynamic_cast<MSDevice_RTSIm*>(item);
                if (device != nullptr) { break; }
            }
            if (device == nullptr || device->myPlatoonID != myPlatoonID || device->myMembers != myMembers) {
                throw InvalidArgument("RTSIm platoon member '" + id + "' must declare the same platoon-id and ordered members");
            }
            MSVehicle* micro = dynamic_cast<MSVehicle*>(candidate);
            if (micro == nullptr || !micro->isOnRoad()) { intact = false; previous = nullptr; continue; }
            if (previous != nullptr && micro->getLeader(std::numeric_limits<double>::max(), false).first != previous) {
                intact = false;
            }
            previous = micro;
        }
        // Membership does not change when the formation breaks. Cooperative
        // feeding resumes only when all assigned members are contiguous again.
        if (myCooperative) {
            cooperativePeer(myLeaderID);
            cooperativePeer(myFrontID);
        }
        return intact;
    }
    if (!myCooperative) { return false; }
    const MSVehicle* veh = static_cast<const MSVehicle*>(&myHolder);
    MSVehicle* leader = cooperativePeer(myLeaderID);
    MSVehicle* front = cooperativePeer(myFrontID);
    if (!veh->isOnRoad() || leader == nullptr || front == nullptr || !leader->isOnRoad() || !front->isOnRoad()
            || veh->getLeader(std::numeric_limits<double>::max(), false).first != front) {
        return false;
    }
    // Legacy leader/front configurations retain their fixed IDs, but the
    // designated leader must occur ahead in the physical predecessor chain.
    const MSVehicle* next = front;
    std::set<const MSVehicle*> visited;
    visited.insert(veh);
    while (next != nullptr && visited.insert(next).second) {
        if (next == leader) { return true; }
        next = next->getLeader(std::numeric_limits<double>::max(), false).first;
    }
    return false;
}

void MSDevice_RTSIm::applyPlexeDesiredSpeed(double speed) {
    MSVehicle* veh = static_cast<MSVehicle*>(&myHolder);
    veh->getCarFollowModel().setParameter(veh, PAR_CC_DESIRED_SPEED, toString(speed, 17));
}

void MSDevice_RTSIm::updateSurface(const MSLane* lane) {
    if (lane == nullptr) { return; }
    const MSEdge& edge = lane->getEdge();
    const std::string road = edge.getParameter("rtsim.road-type", "");
    const std::string value = edge.getParameter("rtsim.fr0", "");
    // Unannotated junction internals retain the previous road's surface.
    if (edge.isInternal() && road.empty() && value.empty()) { return; }
    double coefficient = myDefaultFr0;
    if (!road.empty()) { coefficient = surface(road); }
    if (!value.empty()) {
        coefficient = number(value, "edge rtsim.fr0");
        if (coefficient < 0.) { throw InvalidArgument("RTSIm edge fr0 must be nonnegative"); }
    }
    myFr0 = coefficient;
    if (coefficient >= 0.) { myHolder.getEmissionParameters()->setRollDragCoefficient(coefficient); }
    else { myHolder.getEmissionParameters()->clearRollDragCoefficient(); }
}

bool MSDevice_RTSIm::notifyEnter(SUMOTrafficObject&, MSMoveReminder::Notification, const MSLane* enteredLane) {
    updateSurface(enteredLane != nullptr ? enteredLane : static_cast<MSVehicle&>(myHolder).getLane());
    return true;
}

bool MSDevice_RTSIm::notifyMove(SUMOTrafficObject& veh, double, double, double) {
    const SUMOTime step = MSNet::getInstance()->getCurrentTimeStep();
    if (step != myLastStep) {
        myLastStep = step;
        myLastSlope = veh.getSlope();
        ++mySteps;
    }
    return true;
}

double MSDevice_RTSIm::patchControllerSpeed(double vMin, double vMax) {
    // Optional Krauss-inspired acceleration dawdling for native ACC/CACC.
    // This is an experimental stochastic extension, not a calibrated SAE model.
    if (mySigma <= 0.) { return vMax; }
    const SUMOTime now = MSNet::getInstance()->getCurrentTimeStep();
    if (myNoiseTime < 0 || now - myNoiseTime >= mySigmaStep) {
        myNoiseDraw = RandHelper::rand(&myNoiseRNG);
        myNoiseTime = now;
    }
    const auto& cf = static_cast<MSVehicle&>(myHolder).getCarFollowModel();
    const double candidate = vMax - ACCEL2SPEED(mySigma * cf.getMaxAccel() * myNoiseDraw);
    return MIN2(vMax, MAX2(vMin, candidate));
}

void MSDevice_RTSIm::generateOutput(OutputDevice* out) const {
    if (out == nullptr) { return; }
    const int precision = out->getPrecision();
    out->setPrecision(MAX2(precision, 9));
    out->openTag("rtsim");
    out->writeAttr("automationLevel", myLevel);
    out->writeAttr("sigma", mySigma);
    out->writeAttr("tau", myTau);
    out->writeAttr("cd", myCd);
    out->writeAttr("fr0", myFr0);
    out->writeAttr("platoonSize", mySize);
    out->writeAttr("position", myPosition);
    out->writeAttr("nominalGap", myGap);
    out->writeAttr("lastSlopeDegrees", myLastSlope);
    out->writeAttr("steps", mySteps);
    out->writeAttr("requestedController", myRequestedController);
    out->writeAttr("cooperativeSteps", myCooperativeSteps);
    out->writeAttr("fallbackSteps", myFallbackSteps);
    out->writeAttr("caccSpacing", myCACCSpacing);
    out->writeAttr("platoonID", myPlatoonID);
    out->writeAttr("members", joinedMembers(myMembers));
    out->writeAttr("assignedLeader", myLeaderID);
    out->writeAttr("assignedFront", myFrontID);
    out->writeAttr("formationIntact", myFormationIntact);
    out->closeTag();
    out->setPrecision(precision);
}

std::string MSDevice_RTSIm::getParameter(const std::string& key) const {
    if (key == "cd") { return toString(myCd, 17); }
    if (key == "fr0") { return toString(myFr0, 17); }
    if (key == "sigma") { return toString(mySigma, 17); }
    if (key == "tau") { return toString(myTau, 17); }
    if (key == "slope") { return toString(myLastSlope, 17); }
    if (key == "requestedController") { return myRequestedController; }
    if (key == "cooperativeSteps") { return toString(myCooperativeSteps); }
    if (key == "fallbackSteps") { return toString(myFallbackSteps); }
    if (key == "caccSpacing") { return toString(myCACCSpacing, 17); }
    if (key == "platoonID") { return myPlatoonID; }
    if (key == "members") { return joinedMembers(myMembers); }
    if (key == "assignedLeader") { return myLeaderID; }
    if (key == "assignedFront") { return myFrontID; }
    if (key == "position") { return toString(myPosition); }
    if (key == "platoonSize") { return toString(mySize); }
    if (key == "formationIntact") { return toString(myFormationIntact); }
    if (key == "activeController") {
        const MSVehicle* veh = static_cast<const MSVehicle*>(&myHolder);
        if (myPlexe) {
            const std::string value = veh->getCarFollowModel().getParameter(veh, PAR_ACTIVE_CONTROLLER);
            const int active = StringUtils::toInt(value);
            if (active == Plexe::ACC) { return "ACC"; }
            if (active == Plexe::CACC) { return "CACC"; }
            if (active == Plexe::PLOEG) { return "PLOEG"; }
            return value;
        }
        if (veh->getCarFollowModel().getModelID() == SUMO_TAG_CF_ACC) { return "ACC"; }
        if (veh->getCarFollowModel().getModelID() == SUMO_TAG_CF_CACC) { return "CACC"; }
        if (veh->getCarFollowModel().getModelID() == SUMO_TAG_CF_KRAUSS) { return "Krauss"; }
        return "unchanged";
    }
    throw InvalidArgument("Unsupported RTSIm parameter: " + key);
}

void MSDevice_RTSIm::saveState(OutputDevice&) const {
    throw ProcessError("RTSIm state saving is not implemented; device and random-stream state cannot be omitted");
}

void MSDevice_RTSIm::loadState(const SUMOSAXAttributes&) {
    throw ProcessError("RTSIm state restoration is not implemented");
}
