/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.dev/sumo
// Copyright (C) 2026 RTSIm contributors.
// SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later
/****************************************************************************/
#pragma once
#include <config.h>
#include "MSVehicleDevice.h"
#include <utils/common/SUMOTime.h>
#include <utils/common/RandHelper.h>
#include <vector>

template<class T> class WrappingCommand;

/** Native, fixed-scenario RTSIm prototype. CFD positions/gaps are explicit
 * scenario inputs, not inferred platoon topology. */
class MSDevice_RTSIm : public MSVehicleDevice {
public:
    static void insertOptions(OptionsCont& oc);
    static void buildVehicleDevices(SUMOVehicle& v, std::vector<MSVehicleDevice*>& into);
    static void cleanup();
    ~MSDevice_RTSIm() override;
    const std::string deviceName() const override { return "rtsim"; }
    bool notifyMove(SUMOTrafficObject& veh, double oldPos, double newPos, double newSpeed) override;
    bool notifyEnter(SUMOTrafficObject& veh, MSMoveReminder::Notification reason, const MSLane* enteredLane = nullptr) override;
    void generateOutput(OutputDevice* tripinfoOut) const override;
    std::string getParameter(const std::string& key) const override;
    double patchControllerSpeed(double vMin, double vMax);
    void saveState(OutputDevice& out) const override;
    void loadState(const SUMOSAXAttributes& attrs) override;

private:
    explicit MSDevice_RTSIm(SUMOVehicle& holder);
    void updateSurface(const MSLane* lane);
    void applyPlexeDesiredSpeed(double speed);
    SUMOTime updateCooperativeTopology(SUMOTime currentTime);
    bool isPlatoonFormationIntact() const;
    double myCd = -1.;
    double myDefaultFr0 = -1.;
    double myFr0 = -1.;
    double mySigma = -1.;
    double myTau = -1.;
    double myDesiredSpeed = -1.;
    SUMOTime mySigmaStep = DELTA_T;
    SUMOTime myNoiseTime = -1;
    double myNoiseDraw = 0.;
    SumoRNG myNoiseRNG{"rtsimController"};
    double myLastSlope = 0.;
    int myLevel = -1;
    int mySize = 1;
    int myPosition = 1;
    double myGap = 0.;
    bool myPlexe = false;
    bool myCooperative = false;
    int myRequestedPlexeController = 1;
    std::string myRequestedController = "unchanged";
    std::string myLeaderID;
    std::string myFrontID;
    std::string myPlatoonID;
    std::vector<std::string> myMembers;
    bool myFormationIntact = false;
    double myCACCSpacing = -1.;
    long myCooperativeSteps = 0;
    long myFallbackSteps = 0;
    WrappingCommand<MSDevice_RTSIm>* myTopologyCommand = nullptr;
    std::string myModel;
    std::string myBound;
    std::string myRoadType;

    SUMOTime myLastStep = -1;
    long mySteps = 0;
    MSDevice_RTSIm(const MSDevice_RTSIm&) = delete;
    MSDevice_RTSIm& operator=(const MSDevice_RTSIm&) = delete;
};
