#!/usr/bin/env python3
"""Regression for shared Krauss sigma weakening a Plexe braking command.

An ACC vehicle approaches a stopped vehicle at 25 m/s with a 71 m net gap
(76 m for ballistic integration, whose reference needs extra stopping room).
The reference uses RTSIm tau=0.6 with sigma omitted. Level 5 must preserve
that reference's initial braking, and the default-cadence zero-sigma trace.
Nonzero sigma and longer cadence must never raise the current native ACC
speed request. Both Euler and ballistic integration are exercised.

This is a targeted software regression, not a general controller safety proof.
The test uses SUMO's bundled TraCI and PHEMlight5 passenger-car sample data.
"""
# Eclipse SUMO, Simulation of Urban MObility; https://eclipse.dev/sumo
# Copyright (C) 2026 RTSIm contributors.
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

import argparse
import csv
import json
import math
import os
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

sys.dont_write_bytecode = True
import run_integration as common


STEP = 0.1
PROFILES = ("tau_only", "level5", "sigma_positive")


def network_file(checks, args):
    directory = args.output_dir / "network"
    directory.mkdir(parents=True, exist_ok=True)
    nodes = ET.Element("nodes")
    ET.SubElement(nodes, "node", id="a", x="0", y="0")
    ET.SubElement(nodes, "node", id="b", x="1500", y="0")
    edges = ET.Element("edges")
    ET.SubElement(edges, "edge", id="e", **{"from": "a", "to": "b",
                  "numLanes": "1", "speed": "40"})
    common.write_xml(directory / "nodes.xml", nodes)
    common.write_xml(directory / "edges.xml", edges)
    network = directory / "network.net.xml"
    result = checks.command("netconvert", [args.netconvert, "--node-files", "nodes.xml",
                            "--edge-files", "edges.xml", "--output-file", network], directory)
    checks.check("network.created", result.returncode == 0, stderr=result.stderr)
    return network


def make_routes(path, profile, sigma_step, ballistic):
    root = ET.Element("routes")
    ET.SubElement(root, "vType", id="cc", carFollowModel="CC", lanesCount="1",
                  length="5", minGap="2.5", accel="1.5", decel="4.5",
                  emergencyDecel="9", tauEngine="0.5", maxSpeed="40",
                  speedFactor="1", speedDev="0", emissionClass=common.EMISSION_CLASS)
    ET.SubElement(root, "route", id="r", edges="e")
    ET.SubElement(root, "vehicle", id="leader", type="cc", route="r", depart="0",
                  departPos="200", departSpeed="0")
    follower = ET.SubElement(root, "vehicle", id="follower", type="cc", route="r",
                             depart="0", departPos="119" if ballistic else "124", departSpeed="25")
    parameters = {"has.rtsim.device": "true", "device.rtsim.controller": "ACC",
                  "device.rtsim.desired-speed": 25}
    if profile == "level5":
        parameters["device.rtsim.automation-level"] = 5
    else:
        parameters["device.rtsim.tau"] = 0.6
        if profile == "sigma_positive":
            parameters["device.rtsim.sigma"] = 0.5
    if sigma_step is not None:
        parameters["device.rtsim.sigma-step"] = sigma_step
    common.add_params(follower, parameters)
    common.write_xml(path, root)


def run_case(checks, args, network, profile, sigma_step, ballistic):
    import traci

    name = ("ballistic" if ballistic else "euler") + "_" + profile
    name += "_default" if sigma_step is None else "_held"
    directory = args.output_dir / name
    directory.mkdir(parents=True, exist_ok=True)
    make_routes(directory / "routes.xml", profile, sigma_step, ballistic)
    command = [str(args.sumo), "--net-file", str(network), "--route-files",
               str(directory / "routes.xml"), "--phemlight-path", str(args.phemlight_path),
               "--step-length", str(STEP), "--seed", "42", "--no-step-log", "true",
               "--duration-log.disable", "true", "--collision.action", "warn",
               "--collision-output", str(directory / "collisions.xml"),
               "--error-log", str(directory / "errors.txt"),
               "--step-method.ballistic", str(ballistic).lower()]
    rows = []
    connection = None
    with (directory / "stdout.txt").open("w", encoding="utf-8") as stdout:
        try:
            traci.start(command, label=name, stdout=stdout, numRetries=3)
            connection = traci.getConnection(name)
            connection.simulationStep()
            checks.check(name + ".inserted", set(connection.vehicle.getIDList()) == {"leader", "follower"})
            for vehicle in ("leader", "follower"):
                connection.vehicle.setLaneChangeMode(vehicle, 0)
                connection.vehicle.setParameter(vehicle, "carFollowModel.ccac", "1")
                connection.vehicle.setParameter(vehicle, "carFollowModel.ccds", "25")
            connection.vehicle.setParameter("leader", "carFollowModel.ccfa", "1:0")
            configured = {key: float(connection.vehicle.getParameter("follower", "carFollowModel." + key))
                          for key in ("rtsim.sigma", "rtsim.sigmaStep", "ccaht", "ccac")}
            (directory / "configured.json").write_text(json.dumps(configured, indent=2) + "\n", encoding="utf-8")
            checks.near(name + ".headway", configured["ccaht"], 0.6)
            checks.near(name + ".sigma", configured["rtsim.sigma"],
                        {"tau_only": -1, "level5": 0, "sigma_positive": 0.5}[profile])
            checks.near(name + ".cadence", configured["rtsim.sigmaStep"], sigma_step or STEP)
            for _ in range(100):
                speed_before = connection.vehicle.getSpeed("follower")
                # Query the actual native CC calculation at this same state;
                # no dawdling or actuator is invoked by getStopSpeed. CC's
                # stopSpeed uses its radar and ACC even though the gap argument
                # is zero. This avoids reimplementing the ACC formula in Python.
                native_speed_request = connection.vehicle.getStopSpeed("follower", speed_before, 0)
                connection.simulationStep()
                controller_acceleration = float(connection.vehicle.getParameter(
                    "follower", "carFollowModel.ccsa").split(":")[2])
                rows.append({"time": connection.simulation.getTime(),
                             "speed": connection.vehicle.getSpeed("follower"),
                             "acceleration": connection.vehicle.getAcceleration("follower"),
                             "position": connection.vehicle.getLanePosition("follower"),
                             "net_gap": connection.vehicle.getLanePosition("leader") - 5
                             - connection.vehicle.getLanePosition("follower"),
                             "collisions": connection.simulation.getCollidingVehiclesNumber(),
                             "native_speed_request": native_speed_request,
                             "command_speed": speed_before + STEP * controller_acceleration})
        finally:
            if connection is not None:
                connection.close()
            if rows:
                with (directory / "trace.csv").open("w", newline="", encoding="utf-8") as stream:
                    writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
                    writer.writeheader()
                    writer.writerows(rows)
    checks.commands.append({"name": name, "argv": command, "returncode": 0})
    return name, rows


def check_case(checks, name, rows):
    checks.check(name + ".complete", len(rows) == 100, samples=len(rows))
    checks.check(name + ".finite", all(math.isfinite(value) for row in rows for value in row.values()))
    checks.check(name + ".nonnegative_speed", min(row["speed"] for row in rows) >= 0)
    # ccsa serializes its acceleration with six significant figures; allow
    # 1e-4 m/s for that representation, not an appreciable braking change.
    excess = max(row["command_speed"] - row["native_speed_request"] for row in rows)
    checks.check(name + ".never_raises_native_command", excess <= 1e-4, max_speed_increase=excess)
    checks.near(name + ".initial_deceleration", rows[0]["acceleration"], -4.5)
    checks.check(name + ".no_collision", not any(row["collisions"] for row in rows))
    checks.check(name + ".maintains_minimum_gap", min(row["net_gap"] for row in rows) >= 2.5,
                 minimum_gap=min(row["net_gap"] for row in rows))
    # The leader's first insertion step produces a tiny residual speed through
    # the native actuator; ACC may therefore creep rather than remain at zero.
    checks.check(name + ".slows_to_crawl", rows[-1]["speed"] < 0.25,
                 final_speed=rows[-1]["speed"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sumo", type=Path, required=True)
    parser.add_argument("--netconvert", type=Path, required=True)
    parser.add_argument("--phemlight-path", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path,
                        default=Path(__file__).resolve().parent / "generated" / "braking")
    args = parser.parse_args()
    for key in ("sumo", "netconvert", "phemlight_path", "output_dir"):
        setattr(args, key, getattr(args, key).resolve())
    args.output_dir.mkdir(parents=True, exist_ok=True)
    common.GENERATED = args.output_dir
    os.environ["SUMO_HOME"] = str(common.ROOT)
    sys.path.insert(0, str(common.ROOT / "tools"))
    checks = common.Checks()
    network = network_file(checks, args)
    results = {}
    for ballistic in (False, True):
        for cadence in (None, 0.5):
            for profile in PROFILES:
                key = (ballistic, cadence, profile)
                def exercise(key=key):
                    mode, interval, kind = key
                    name, rows = run_case(checks, args, network, kind, interval, mode)
                    results[key] = rows
                    check_case(checks, name, rows)
                checks.group(str(key), exercise)
        def zero_parity(mode=ballistic):
            reference = results[(mode, None, "tau_only")]
            zero = results[(mode, None, "level5")]
            difference = max(abs(a[field] - b[field]) for a, b in zip(reference, zero)
                             for field in ("speed", "acceleration", "position", "net_gap"))
            checks.check(("ballistic" if mode else "euler") + ".level5_default_trace_matches",
                         difference < 1e-9, maximum_difference=difference)
        checks.group("zero_parity_" + str(ballistic), zero_parity)
    passed = sum(item["passed"] for item in checks.results)
    report = {"passed": passed, "failed": len(checks.results) - passed,
              "checks": checks.results, "commands": checks.commands}
    (args.output_dir / "braking_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"passed": report["passed"], "failed": report["failed"],
                      "report": str(args.output_dir / "braking_report.json")}))
    return 1 if report["failed"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
