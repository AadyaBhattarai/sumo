#!/usr/bin/env python3
"""Mechanical tests of optional native RTSIm ACC/CACC stochastic extensions.

The tests use synthetic two-vehicle traffic and bundled passenger-car emissions
data. They do not validate automation-level calibrations or physical CFD data.
CC/ACC zero-noise checks use an equipped vehicle with sigma unset as their
reference: unequipped CC uses a different (human-driver) controller mode.
CACC direct controls set both tau and tauCACCToACC because SUMO's setTau API
updates both the cooperative and fallback ACC headways.
CC applies the shared Krauss sigma speed transformation before its existing
engine; this does not imply identical motion to a complete Krauss vehicle.
The native ACC/CACC tests retain coverage of their earlier experimental path.
Focused runtime checks also use SUMO's bundled TraCI module to exercise actual
Plexe CACC (carFollowModel=CC, controller=2) at fixed spacing, and a sigma-step
change at simulation time 0.9s. No CACC time-headway policy is introduced.
"""
# Eclipse SUMO, Simulation of Urban MObility; https://eclipse.dev/sumo
# Copyright (C) 2026 RTSIm contributors.
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

import argparse
import datetime
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

sys.dont_write_bytecode = True
import run_integration as common


DIRECTORY = Path(__file__).resolve().parent / "generated" / "controllers"
common.GENERATED = DIRECTORY


def vehicle_specs(noise=False, equal_tau=False):
    return [
        {"id": "leader", "pos": 80, "speed": 10, "sigma": 0,
         "tau": 1 if equal_tau else 0.8},
        # A 25m net gap becomes shorter than the 1.4s target during acceleration,
        # so the tau comparison exercises following rather than free flow only.
        {"id": "follower", "pos": 50, "speed": 10,
         "sigma": 0.55 if noise else 0, "tau": 1 if equal_tau else 1.4},
    ]


def make_routes(path, model, specifications, equipped, sigma_unset=False, sigma_step=None):
    root = ET.Element("routes")
    attributes = dict(carFollowModel=model, emissionClass=common.EMISSION_CLASS,
                      accel="1", decel="4.5", emergencyDecel="9", length="5",
                      minGap="2.5", maxSpeed="20", speedFactor="1", speedDev="0")
    if model == "CC":
        attributes["lanesCount"] = "1"
    if equipped:
        ET.SubElement(root, "vType", id="shared", tau="1", **attributes)
    else:
        for specification in specifications:
            kind = ET.SubElement(root, "vType", id="type_" + specification["id"],
                                 tau=str(specification["tau"]), **attributes)
            if model == "CACC":
                kind.set("tauCACCToACC", str(specification["tau"]))
            common.add_params(kind, {"airDragCoefficient": 0.6, "rollDragCoefficient": 0.01})
    ET.SubElement(root, "route", id="route", edges="e0 e1")
    for specification in specifications:
        element = ET.SubElement(root, "vehicle", id=specification["id"],
                                type="shared" if equipped else "type_" + specification["id"],
                                route="route", depart="0", departLane="0",
                                departPos=str(specification["pos"]), departSpeed=str(specification["speed"]))
        if equipped:
            parameters = {"has.rtsim.device": "true", "device.rtsim.controller": specification.get("controller", "ACC" if model == "CC" else model),
                          "device.rtsim.tau": specification["tau"], "device.rtsim.cd": 0.6,
                          "device.rtsim.fr0": 0.01}
            if sigma_step is not None:
                parameters["device.rtsim.sigma-step"] = sigma_step
            if not sigma_unset:
                parameters["device.rtsim.sigma"] = specification["sigma"]
            if model == "CC":
                parameters["device.rtsim.desired-speed"] = 20
                for key in ("leader", "front", "cacc-spacing"):
                    if key in specification:
                        parameters["device.rtsim." + key] = specification[key]
            common.add_params(element, parameters)
    common.write_xml(path, root)


def run(checks, args, network, name, model, specs, equipped=True, seed=42, sigma_unset=False,
        step=0.1, sigma_step=None, reject=None):
    directory = DIRECTORY / name
    directory.mkdir(parents=True, exist_ok=True)
    make_routes(directory / "routes.rou.xml", model, specs, equipped, sigma_unset, sigma_step)
    command = [args.sumo, "--net-file", network, "--route-files", "routes.rou.xml",
               "--phemlight-path", args.phemlight_path, "--step-length", str(step),
               "--seed", str(seed), "--end", "200", "--no-step-log", "true",
               "--duration-log.disable", "true", "--precision", "10",
               "--device.emissions.probability", "1", "--emission-output", "emissions.xml",
               "--emission-output.precision", "10", "--fcd-output", "fcd.xml",
               "--fcd-output.acceleration", "true", "--tripinfo-output", "tripinfo.xml",
               "--collision-output", "collisions.xml", "--collision.action", "warn"]
    result = checks.command(name, command, directory)
    if reject is not None:
        diagnostic = result.stdout + result.stderr
        checks.check(name + ".rejected", result.returncode != 0, returncode=result.returncode)
        checks.check(name + ".diagnostic", "RTSIm" in diagnostic and reject in diagnostic,
                     expected=reject, diagnostic=diagnostic[-2500:])
        return None
    checks.check(name + ".run", result.returncode == 0, stderr=result.stderr)
    trips = {item.get("id"): item for item in ET.parse(directory / "tripinfo.xml").getroot().findall("tripinfo")}
    checks.check(name + ".all_arrived", set(trips) == {spec["id"] for spec in specs},
                 vehicles=sorted(trips))
    collisions = ET.parse(directory / "collisions.xml").getroot().findall("collision")
    checks.check(name + ".no_collisions", not collisions,
                 collisions=[dict(item.attrib) for item in collisions])
    checks.check(name + ".no_teleports", "teleport" not in result.stderr.lower(),
                 stderr=result.stderr)
    fcd = common.timestep_rows(directory / "fcd.xml")
    speeds = [float(row["speed"]) for row in fcd.values()]
    checks.check(name + ".nonnegative_finite_speeds", bool(speeds) and all(
        math.isfinite(value) and value >= 0 for value in speeds), minimum_speed=min(speeds, default=None))
    checks.check(name + ".single_lane", all(row["lane"] in ("e0_0", "e1_0") for row in fcd.values()))
    for specification in specs:
        state = trips[specification["id"]].find("rtsim")
        checks.check(name + ".device_presence." + specification["id"],
                     (state is not None) == equipped)
        if equipped:
            checks.near(name + ".tau." + specification["id"], float(state.get("tau")), specification["tau"])
            checks.near(name + ".sigma." + specification["id"], float(state.get("sigma")),
                        -1 if sigma_unset else specification["sigma"])
    return {"directory": directory, "trips": trips, "fcd": fcd,
            "emissions": common.timestep_rows(directory / "emissions.xml")}


def exact_trajectories(checks, name, left, right, vehicle_id=None):
    a = {key: row for key, row in left["fcd"].items() if vehicle_id is None or key[1] == vehicle_id}
    b = {key: row for key, row in right["fcd"].items() if vehicle_id is None or key[1] == vehicle_id}
    checks.check(name + ".keys", bool(a) and a.keys() == b.keys(),
                 left_samples=len(a), right_samples=len(b))
    for field in ("speed", "pos", "acceleration", "x", "y"):
        differences = [abs(float(a[key][field]) - float(b[key][field])) for key in a]
        checks.check(name + ".exact." + field, all(value == 0 for value in differences),
                     max_absolute_difference=max(differences))


def different_trajectory(checks, name, left, right, vehicle_id="follower"):
    keys = [key for key in left["fcd"] if key[1] == vehicle_id and key in right["fcd"]]
    checks.check(name + ".samples", len(keys) > 50, samples=len(keys))
    maximum = max(abs(float(left["fcd"][key]["speed"]) - float(right["fcd"][key]["speed"])) for key in keys)
    checks.check(name + ".changed", maximum > 1e-5, max_speed_difference=maximum)


def run_plexe_runtime(checks, args, network, name, follower_sigma, cacc=False, change_interval=False,
                      ploeg=False, headway=1.4):
    # Import the checked-out SUMO client; no separately installed package or
    # external service is required. All communication is with this local run.
    tools_path = str(common.ROOT / "tools")
    if tools_path not in sys.path:
        sys.path.insert(0, tools_path)
    import traci

    directory = DIRECTORY / name
    directory.mkdir(parents=True, exist_ok=True)
    make_routes(directory / "routes.rou.xml", "CC", vehicle_specs(), True)
    command = [str(args.sumo), "--net-file", str(network), "--route-files", str(directory / "routes.rou.xml"),
               "--phemlight-path", str(args.phemlight_path), "--step-length", "0.1", "--seed", "42",
               "--end", "25", "--no-step-log", "true", "--duration-log.disable", "true",
               "--precision", "10", "--fcd-output", str(directory / "fcd.xml"),
               "--fcd-output.acceleration", "true", "--collision-output", str(directory / "collisions.xml"),
               "--collision.action", "warn", "--error-log", str(directory / "sumo.errors.txt")]
    connection = None
    operations = []
    try:
        with (directory / "sumo.stdout.txt").open("w", encoding="utf-8") as stdout:
            traci.start(command, label=name, stdout=stdout, numRetries=5)
            connection = traci.getConnection(name)
            connection.simulationStep()
            checks.check(name + ".vehicles_inserted", set(connection.vehicle.getIDList()) == {"leader", "follower"})

            def set_parameter(key, value):
                connection.vehicle.setParameter("follower", "carFollowModel." + key, str(value))
                operations.append({"time": connection.simulation.getTime(), "vehicle": "follower",
                                   "parameter": key, "value": str(value)})

            if cacc or ploeg:
                # CC_Const.h: ccac=2 is Plexe CACC; ccaf uses colon-delimited
                # leader and front IDs. The leader also acts as the predecessor
                # in this two-vehicle platoon. CC's internal auto-feed supplies
                # actual speed and acceleration every controller evaluation.
                if ploeg:
                    set_parameter("ccph", headway)
                else:
                    set_parameter("ccsp", 25)
                set_parameter("ccaf", "1:leader:leader")
                active_mode = 4 if ploeg else 2
                set_parameter("ccac", active_mode)
                checks.near(name + ".actual_plexe_controller", float(connection.vehicle.getParameter(
                    "follower", "carFollowModel.ccac")), active_mode)
                if cacc:
                    checks.near(name + ".fixed_spacing", float(connection.vehicle.getParameter(
                        "follower", "carFollowModel.ccsp")), 25)

            if change_interval:
                while connection.simulation.getTime() < 0.9 - 1e-9:
                    connection.simulationStep()
                checks.near(name + ".interval_change_time", connection.simulation.getTime(), 0.9)
                set_parameter("rtsim.sigmaStep", 1.0)
                checks.near(name + ".interval_applied", float(connection.vehicle.getParameter(
                    "follower", "carFollowModel.rtsim.sigmaStep")), 1.0)
            set_parameter("rtsim.sigma", follower_sigma)
            checks.near(name + ".sigma_applied", float(connection.vehicle.getParameter(
                "follower", "carFollowModel.rtsim.sigma")), follower_sigma)
            while connection.simulation.getTime() < 20 - 1e-9:
                connection.simulationStep()
            if cacc or ploeg:
                checks.near(name + ".cacc_remains_active", float(connection.vehicle.getParameter(
                    "follower", "carFollowModel.ccac")), active_mode)
            connection.close()
            connection = None
    finally:
        if connection is not None:
            connection.close()
        (directory / "runtime_commands.json").write_text(json.dumps(operations, indent=2) + "\n", encoding="utf-8")

    checks.commands.append({"name": name, "argv": command, "cwd": str(Path.cwd()),
                            "returncode": 0, "control": "bundled local TraCI", "runtime_commands": operations})
    rows = common.timestep_rows(directory / "fcd.xml")
    speeds = [float(row["speed"]) for row in rows.values()]
    checks.check(name + ".finite_nonnegative_speeds", bool(speeds) and all(
        math.isfinite(speed) and speed >= 0 for speed in speeds), minimum_speed=min(speeds, default=None))
    collisions = ET.parse(directory / "collisions.xml").getroot().findall("collision")
    checks.check(name + ".no_collisions", not collisions,
                 collisions=[dict(item.attrib) for item in collisions])
    errors_path = directory / "sumo.errors.txt"
    errors = errors_path.read_text(encoding="utf-8", errors="replace") if errors_path.exists() else ""
    checks.check(name + ".no_teleports", "teleport" not in errors.lower(), diagnostic=errors)
    return {"directory": directory, "fcd": rows}


def test_plexe_runtime(checks, args, network):
    def interval_boundary():
        zero = run_plexe_runtime(checks, args, network, "cc_runtime_interval_zero", 0, change_interval=True)
        noisy = run_plexe_runtime(checks, args, network, "cc_runtime_interval_noise", 0.55, change_interval=True)
        repeated = run_plexe_runtime(checks, args, network, "cc_runtime_interval_repeat", 0.55, change_interval=True)
        different_trajectory(checks, "cc.runtime_interval_reaches_noise_sampling", zero, noisy)
        exact_trajectories(checks, "cc.runtime_interval_reproducible", noisy, repeated)
        exact_trajectories(checks, "cc.runtime_interval_leader_unchanged", zero, noisy, "leader")
    checks.group("cc_runtime_interval", interval_boundary)

    def actual_cacc():
        zero = run_plexe_runtime(checks, args, network, "plexe_cacc_zero", 0, cacc=True)
        noisy = run_plexe_runtime(checks, args, network, "plexe_cacc_noise", 0.55, cacc=True)
        repeated = run_plexe_runtime(checks, args, network, "plexe_cacc_repeat", 0.55, cacc=True)
        different_trajectory(checks, "plexe_cacc.own_sigma_effect", zero, noisy)
        exact_trajectories(checks, "plexe_cacc.sigma_reproducible", noisy, repeated)
        exact_trajectories(checks, "plexe_cacc.leader_unchanged", zero, noisy, "leader")
    checks.group("plexe_cacc_sigma", actual_cacc)

    def actual_ploeg():
        zero = run_plexe_runtime(checks, args, network, "plexe_ploeg_zero", 0, ploeg=True)
        noisy = run_plexe_runtime(checks, args, network, "plexe_ploeg_noise", 0.55, ploeg=True)
        repeated = run_plexe_runtime(checks, args, network, "plexe_ploeg_repeat", 0.55, ploeg=True)
        lower_h = run_plexe_runtime(checks, args, network, "plexe_ploeg_lower_h", 0, ploeg=True, headway=0.8)
        different_trajectory(checks, "plexe_ploeg.own_sigma_effect", zero, noisy)
        different_trajectory(checks, "plexe_ploeg.native_headway_effect", zero, lower_h)
        exact_trajectories(checks, "plexe_ploeg.sigma_reproducible", noisy, repeated)
        exact_trajectories(checks, "plexe_ploeg.leader_unchanged", zero, noisy, "leader")
    checks.group("plexe_ploeg_sigma_headway", actual_ploeg)


def test_native_platoons(checks, args, network):
    def specifications(controller, noise=False, tail_tau=1.4):
        result = [
            {"id": "leader", "pos": 110, "speed": 10, "sigma": 0, "tau": 1, "controller": "ACC"},
            {"id": "middle", "pos": 80, "speed": 10, "sigma": 0.2 if noise else 0, "tau": 0.8,
             "controller": controller, "leader": "leader", "front": "leader"},
            {"id": "follower", "pos": 50, "speed": 10, "sigma": 0.55 if noise else 0, "tau": tail_tau,
             "controller": controller, "leader": "leader", "front": "middle"},
        ]
        if controller == "CACC":
            for follower in result[1:]:
                follower["cacc-spacing"] = 25
        return result

    def topology_state(name, result, controller):
        for vehicle_id in ("middle", "follower"):
            state = result["trips"][vehicle_id].find("rtsim")
            checks.check(name + ".requested_controller." + vehicle_id,
                         state.get("requestedController") == controller, actual=state.get("requestedController"))
            checks.check(name + ".cooperative_steps." + vehicle_id,
                         int(state.get("cooperativeSteps", "0")) > 0, count=state.get("cooperativeSteps"))
            checks.check(name + ".fallback_after_peer_exit." + vehicle_id,
                         int(state.get("fallbackSteps", "0")) > 0, count=state.get("fallbackSteps"))
            if controller == "CACC":
                checks.near(name + ".fixed_spacing." + vehicle_id, float(state.get("caccSpacing")), 25)

    for controller in ("CACC", "PLOEG"):
        def group(mode=controller):
            prefix = "device_" + mode.lower()
            zero = run(checks, args, network, prefix + "_zero", "CC", specifications(mode))
            noisy = run(checks, args, network, prefix + "_noise", "CC", specifications(mode, noise=True))
            repeated = run(checks, args, network, prefix + "_repeat", "CC", specifications(mode, noise=True))
            topology_state(prefix + ".zero", zero, mode)
            topology_state(prefix + ".noise", noisy, mode)
            different_trajectory(checks, prefix + ".heterogeneous_sigma_effect", zero, noisy)
            exact_trajectories(checks, prefix + ".reproducible", noisy, repeated)
            exact_trajectories(checks, prefix + ".leader_unchanged", zero, noisy, "leader")
            common.compare_runs(checks, prefix + ".repeat_outputs", noisy, repeated)
            if mode == "PLOEG":
                lower_tau = run(checks, args, network, prefix + "_lower_tau", "CC", specifications(mode, tail_tau=0.8))
                different_trajectory(checks, prefix + ".native_tau_effect", zero, lower_tau)
        checks.group("native_device_" + controller.lower(), group)


def test_automation_levels(checks, args, network):
    tools_path = str(common.ROOT / "tools")
    if tools_path not in sys.path:
        sys.path.insert(0, tools_path)
    import traci

    sigma_values = (0.5, 0.4, 0.3, 0.2, 0.0, 0.0)
    tau_values = (1.0, 0.95, 0.9, 0.8, 0.7, 0.6)

    for controller in ("ACC", "PLOEG"):
        def group(mode=controller):
            name = "levels_" + mode.lower()
            directory = DIRECTORY / name
            directory.mkdir(parents=True, exist_ok=True)
            root = ET.Element("routes")
            ET.SubElement(root, "vType", id="shared", carFollowModel="CC", lanesCount="1",
                          emissionClass=common.EMISSION_CLASS, accel="1", decel="4.5",
                          emergencyDecel="9", length="5", minGap="2.5", maxSpeed="20",
                          speedFactor="1", speedDev="0", tau="1")
            ET.SubElement(root, "route", id="route", edges="e0 e1")
            anchor = ET.SubElement(root, "vehicle", id="anchor", type="shared", route="route",
                                   depart="0", departPos="260", departSpeed="10")
            common.add_params(anchor, {"has.rtsim.device": "true", "device.rtsim.controller": "ACC",
                                        "device.rtsim.desired-speed": 20, "device.rtsim.sigma": 0,
                                        "device.rtsim.tau": 1})
            for level in range(6):
                item = ET.SubElement(root, "vehicle", id="level%d" % level, type="shared", route="route",
                                     depart="0", departPos=str(230 - 30 * level), departSpeed="10")
                # Level selection is the normal interface under test. No sigma
                # or tau overrides appear on these six vehicles.
                parameters = {"has.rtsim.device": "true", "device.rtsim.controller": mode,
                              "device.rtsim.desired-speed": 20, "device.rtsim.automation-level": level}
                if mode == "PLOEG":
                    parameters.update({"device.rtsim.leader": "anchor",
                                       "device.rtsim.front": "anchor" if level == 0 else "level%d" % (level - 1)})
                common.add_params(item, parameters)
            common.write_xml(directory / "routes.rou.xml", root)
            command = [str(args.sumo), "--net-file", str(network), "--route-files", str(directory / "routes.rou.xml"),
                       "--phemlight-path", str(args.phemlight_path), "--step-length", "0.1", "--seed", "42",
                       "--end", "5", "--no-step-log", "true", "--duration-log.disable", "true", "--precision", "10",
                       "--tripinfo-output", str(directory / "tripinfo.xml"), "--tripinfo-output.write-unfinished", "true",
                       "--collision-output", str(directory / "collisions.xml"), "--collision.action", "warn"]
            connection = None
            mappings = []
            try:
                with (directory / "sumo.stdout.txt").open("w", encoding="utf-8") as stdout:
                    traci.start(command, label=name, stdout=stdout, numRetries=5)
                    connection = traci.getConnection(name)
                    for unused in range(5):
                        connection.simulationStep()
                    checks.check(name + ".all_levels_present", set(connection.vehicle.getIDList()) ==
                                 {"anchor", *("level%d" % level for level in range(6))})
                    for level in range(6):
                        vehicle_id = "level%d" % level
                        prefix = name + ".level%d" % level
                        getter = lambda key: float(connection.vehicle.getParameter(vehicle_id, "carFollowModel." + key))
                        actual_sigma = getter("rtsim.sigma")
                        actual_acc_tau = getter("ccaht")
                        actual_mode = getter("ccac")
                        checks.near(prefix + ".actual_sigma", actual_sigma, sigma_values[level], tolerance=1e-12)
                        checks.near(prefix + ".actual_acc_headway", actual_acc_tau, tau_values[level], tolerance=1e-12)
                        checks.near(prefix + ".actual_controller", actual_mode, 4 if mode == "PLOEG" else 1)
                        record = {"level": level, "actual_sigma": actual_sigma,
                                  "actual_acc_headway": actual_acc_tau, "actual_controller": actual_mode}
                        if mode == "PLOEG":
                            actual_h = getter("ccph")
                            checks.near(prefix + ".actual_ploeg_headway", actual_h, tau_values[level], tolerance=1e-12)
                            record["actual_ploeg_headway"] = actual_h
                        mappings.append(record)
                    while connection.simulation.getTime() < 2 - 1e-9:
                        connection.simulationStep()
                    connection.close()
                    connection = None
            finally:
                if connection is not None:
                    connection.close()
            (directory / "actual_level_mapping.json").write_text(json.dumps(mappings, indent=2) + "\n", encoding="utf-8")
            checks.commands.append({"name": name, "argv": command, "cwd": str(Path.cwd()),
                                    "returncode": 0, "control": "bundled local TraCI"})
            trips = {item.get("id"): item for item in ET.parse(directory / "tripinfo.xml").getroot().findall("tripinfo")}
            for level in range(6):
                state = trips["level%d" % level].find("rtsim")
                checks.near(name + ".reported_level%d" % level, float(state.get("automationLevel")), level)
            collisions = ET.parse(directory / "collisions.xml").getroot().findall("collision")
            checks.check(name + ".no_collisions", not collisions)
        checks.group("automation_levels_" + controller.lower(), group)


def test_model(checks, args, network, model):
    prefix = model.lower()
    runs = {}

    def baseline():
        runs["zero"] = run(checks, args, network, prefix + "_zero", model, vehicle_specs())
        if model == "CC":
            reference = run(checks, args, network, prefix + "_unset", model, vehicle_specs(), sigma_unset=True)
            reference_name = prefix + ".zero_equals_sigma_unset"
        else:
            reference = run(checks, args, network, prefix + "_direct", model, vehicle_specs(), equipped=False)
            reference_name = prefix + ".zero_equals_no_device_direct_tau"
        exact_trajectories(checks, reference_name, runs["zero"], reference)
        common.compare_runs(checks, prefix + ".baseline_emissions_and_trips", runs["zero"], reference)
    checks.group(prefix + "_baseline", baseline)

    def tau_effect():
        equal = run(checks, args, network, prefix + "_equal_tau", model, vehicle_specs(equal_tau=True))
        different_trajectory(checks, prefix + ".heterogeneous_tau_affects_follower", runs["zero"], equal)
    checks.group(prefix + "_tau", tau_effect)

    def noise():
        runs["noise"] = run(checks, args, network, prefix + "_noise", model, vehicle_specs(noise=True))
        different_trajectory(checks, prefix + ".own_follower_sigma_effect", runs["zero"], runs["noise"])
        exact_trajectories(checks, prefix + ".leader_unchanged_by_follower_sigma", runs["zero"], runs["noise"], "leader")
    checks.group(prefix + "_noise", noise)

    def repeat():
        repeated = run(checks, args, network, prefix + "_repeat", model, vehicle_specs(noise=True))
        exact_trajectories(checks, prefix + ".same_seed_reproducible", runs["noise"], repeated)
        common.compare_runs(checks, prefix + ".same_seed_outputs", runs["noise"], repeated)
    checks.group(prefix + "_repeat", repeat)

    def new_seed():
        changed = run(checks, args, network, prefix + "_seed43", model, vehicle_specs(noise=True), seed=43)
        different_trajectory(checks, prefix + ".different_seed_effect", runs["noise"], changed)
        exact_trajectories(checks, prefix + ".leader_unchanged_by_seed", runs["noise"], changed, "leader")
    checks.group(prefix + "_seed", new_seed)

    def bad_interval():
        run(checks, args, network, prefix + "_bad_interval", model, vehicle_specs(noise=True),
            sigma_step=0.15, reject="sigma-step")
    checks.group(prefix + "_bad_interval", bad_interval)

    def zero_nondividing_step():
        run(checks, args, network, prefix + "_zero_step03", model, vehicle_specs()[:1],
            step=0.3, sigma_step=None)
    checks.group(prefix + "_zero_step03", zero_nondividing_step)

    if model == "CC":
        def long_interval():
            zero = run(checks, args, network, prefix + "_long_zero", model,
                       vehicle_specs(), sigma_step=0.5)
            noisy = run(checks, args, network, prefix + "_long_noise", model,
                        vehicle_specs(noise=True), sigma_step=0.5)
            repeated = run(checks, args, network, prefix + "_long_repeat", model,
                           vehicle_specs(noise=True), sigma_step=0.5)
            different_trajectory(checks, prefix + ".long_interval_sigma_effect", zero, noisy)
            exact_trajectories(checks, prefix + ".long_interval_leader_unchanged", zero, noisy, "leader")
            exact_trajectories(checks, prefix + ".long_interval_repeat", noisy, repeated)
            common.compare_runs(checks, prefix + ".long_interval_repeat_outputs", noisy, repeated)
        checks.group(prefix + "_long_interval", long_interval)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sumo", type=Path, required=True)
    parser.add_argument("--netconvert", type=Path, required=True)
    parser.add_argument("--phemlight-path", type=Path, default=common.ROOT / "data" / "emissions" / "PHEMlight5")
    parser.add_argument("--models", nargs="+", choices=("ACC", "CACC", "CC"), default=("ACC", "CACC", "CC"))
    parser.add_argument("--skip-native-platoons", action="store_true",
                        help="Run controller tests while native topology device changes are still being built.")
    parser.add_argument("--levels-only", action="store_true",
                        help="Only check all six level presets against actual CC/ACC and PLOEG controller parameters.")
    args = parser.parse_args()
    args.sumo = args.sumo.resolve()
    args.netconvert = args.netconvert.resolve()
    args.phemlight_path = args.phemlight_path.resolve()
    DIRECTORY.mkdir(parents=True, exist_ok=True)
    checks = common.Checks()
    try:
        checks.check("preflight.sumo", args.sumo.is_file(), path=str(args.sumo))
        checks.check("preflight.netconvert", args.netconvert.is_file(), path=str(args.netconvert))
        checks.check("preflight.phemlight", (args.phemlight_path / "PC_EU4_D_MW.PHEMLight.veh").is_file())
        network = common.make_network(checks, args, "network")
        if args.levels_only:
            test_automation_levels(checks, args, network)
        else:
            for model in args.models:
                test_model(checks, args, network, model)
            if "CC" in args.models:
                test_plexe_runtime(checks, args, network)
                if not args.skip_native_platoons:
                    test_native_platoons(checks, args, network)
                    test_automation_levels(checks, args, network)
    except Exception as error:
        if not checks.results or checks.results[-1]["passed"]:
            checks.results.append({"name": "controller_harness", "passed": False, "error": str(error)})
        print("FAIL:", error)
    failed = sum(not result["passed"] for result in checks.results)
    report = {"created_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
              "scope": "Mechanical synthetic ACC/CACC/CC-ACC stochastic-extension tests; not research validation.",
              "cc_reference": "Equipped CC/ACC with sigma unset, because unequipped CC selects another controller mode.",
              "cc_sigma_scope": "Shared Krauss sigma speed transformation before the existing CC engine. Motion is not claimed identical to the complete Krauss model. Zero-versus-unset parity uses the default simulation-step cadence only.",
              "native_controller_scope": "ACC/CACC smoke tests cover their earlier experimental extension; they do not establish exact Krauss semantics.",
              "cacc_reference": "Direct vType tau and tauCACCToACC both match device tau, reproducing SUMO setTau API semantics.",
              "models": args.models, "sumo": str(args.sumo), "main_step_length": 0.1,
              "zero_noise_additional_step_length": 0.3,
              "seeds": [42, 43], "main_sigma_step": "omitted; defaults to simulation timestep",
              "cc_additional_sigma_step": 0.5,
              "cc_runtime_interval_test": "At t=0.9s, change sigmaStep to 1.0s; confirm noise effect and reproducibility.",
              "plexe_cacc_test": "carFollowModel=CC follower switched to Plexe controller 2 with 25m constant spacing and internal predecessor/leader auto-feed; sigma only, no tau policy change.",
              "plexe_ploeg_test": "Existing Plexe controller 4 uses its own time-headway h; tests cover shared Krauss sigma and native headway changes.",
              "native_device_platoon_test": "Three carFollowModel=CC vehicles, explicit leader/front IDs, ideal internal auto-feed, cooperative steps then ACC fallback after peer exit. PLOEG tau maps to native h; PATH CACC retains fixed spacing.",
              "native_device_platoons_executed": not args.levels_only and "CC" in args.models and not args.skip_native_platoons,
              "automation_level_mapping_test": "Six per-vehicle levels sharing one CC vType; sigma and ACC headway queried from actual controller state, plus native Ploeg h through ccph, not just device labels.",
              "levels_only": args.levels_only,
              "passed": len(checks.results) - failed, "failed": failed,
              "assertions": checks.results, "commands": checks.commands}
    path = DIRECTORY / ("automation_levels_report.json" if args.levels_only else "controller_report.json")
    path.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print("%d assertions passed; %d failed. Report: %s" % (report["passed"], failed, path))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
