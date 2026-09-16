#!/usr/bin/env python3
"""Regression tests for fixed platoon identity, topology fallback and Krauss input.

Synthetic cut-ins are imposed with TraCI moveTo to test device state transitions,
not lane-change or radio models. Cd is looked up once from a synthetic CFD table
and must remain unchanged when the assigned formation breaks or reforms.
"""
# Eclipse SUMO, Simulation of Urban MObility; https://eclipse.dev/sumo
# Copyright (C) 2026 RTSIm contributors.
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

sys.dont_write_bytecode = True
import run_integration as common

CD = {"A": 0.6, "B": 0.45, "C": 0.5}


def network(checks, args):
    directory = args.output_dir / "network"
    directory.mkdir(parents=True, exist_ok=True)
    nodes = ET.Element("nodes")
    ET.SubElement(nodes, "node", id="n0", x="0", y="0")
    ET.SubElement(nodes, "node", id="n1", x="3000", y="0")
    edges = ET.Element("edges")
    ET.SubElement(edges, "edge", id="road", **{"from": "n0", "to": "n1"},
                  numLanes="2", speed="20")
    common.write_xml(directory / "nodes.xml", nodes)
    common.write_xml(directory / "edges.xml", edges)
    result = checks.command("netconvert", [args.netconvert, "-n", "nodes.xml", "-e", "edges.xml",
                            "-o", "network.net.xml", "--no-internal-links", "true"], directory)
    checks.check("network.created", result.returncode == 0, stderr=result.stderr)
    return directory / "network.net.xml"


def routes(path, mode="PLOEG", group="P1", late=False, legacy=False, wrong_leader=False,
           overrides=None, missing_device=None):
    root = ET.Element("routes")
    ET.SubElement(root, "vType", id="cc", carFollowModel="CC", lanesCount="2",
                  emissionClass=common.EMISSION_CLASS, accel="1", decel="4.5", emergencyDecel="9",
                  length="5", minGap="2.5", maxSpeed="20", speedFactor="1", speedDev="0")
    ET.SubElement(root, "route", id="r", edges="road")
    specifications = [("A", 300, 0, 2 if late else 0), ("B", 200, 0, 0),
                      ("C", 100, 0, 0), ("X", 150 if not wrong_leader else 50, 1, 0)]
    for vehicle_id, position, lane, depart in sorted(specifications, key=lambda item: item[3]):
        vehicle = ET.SubElement(root, "vehicle", id=vehicle_id, type="cc", route="r",
                                depart=str(depart), departPos=str(position), departSpeed="10",
                                departLane=str(lane))
        if vehicle_id == missing_device:
            continue
        params = {"has.rtsim.device": "true", "device.rtsim.controller": "ACC",
                  "device.rtsim.desired-speed": 10, "device.rtsim.sigma": 0,
                  "device.rtsim.tau": 0.8, "device.rtsim.cd": 0.6}
        if vehicle_id != "X":
            params["device.rtsim.controller"] = mode if vehicle_id != "A" or not legacy else "ACC"
            if mode == "CACC" and (vehicle_id != "A" or not legacy):
                params["device.rtsim.cacc-spacing"] = 95
            if legacy:
                if vehicle_id != "A":
                    params["device.rtsim.leader"] = "X" if wrong_leader else "A"
                    params["device.rtsim.front"] = "A" if vehicle_id == "B" else "B"
            else:
                params.update({"device.rtsim.platoon-id": group, "device.rtsim.members": "A B C",
                               "device.rtsim.model": "synthetic", "device.rtsim.gap": 10})
                del params["device.rtsim.cd"]
        params.update((overrides or {}).get(vehicle_id, {}))
        common.add_params(vehicle, {key: value for key, value in params.items() if value is not None})
    common.write_xml(path, root)


def command(args, net, directory):
    (directory / "cfd.csv").write_text(
        "model_id,platoon_size,gap_m,position,cd_lower,cd_upper\n"
        + "".join("synthetic,3,10,%d,%s,%s\n" % (index, value, value)
                  for index, value in enumerate(CD.values(), 1)), encoding="utf-8")
    return [str(args.sumo), "--net-file", str(net), "--route-files", str(directory / "routes.xml"),
            "--phemlight-path", str(args.phemlight_path), "--step-length", "0.1",
            "--device.rtsim.cfd-file", str(directory / "cfd.csv"), "--seed", "42",
            "--end", "8", "--no-step-log", "true", "--duration-log.disable", "true",
            "--tripinfo-output", str(directory / "tripinfo.xml"),
            "--tripinfo-output.write-unfinished", "true", "--collision.action", "warn",
            "--collision-output", str(directory / "collisions.xml")]


def state(checks, connection, name, mode, intact, members=("A", "B", "C"), group="P1", manual_models=None):
    result = {}
    expected_front = {"A": "", "B": "A", "C": "B"}
    for member in members:
        record = {key: connection.vehicle.getParameter(member, "device.rtsim." + key)
                  for key in ("platoonID", "members", "assignedLeader", "assignedFront", "position",
                              "platoonSize", "formationIntact", "activeController", "cd")}
        expected = {"platoonID": group, "members": "A B C", "assignedLeader": "A",
                    "assignedFront": expected_front[member], "position": str("ABC".index(member) + 1),
                    "platoonSize": "3", "activeController": mode if intact and member != "A" else "ACC"}
        if member in (manual_models or {}):
            expected["activeController"] = manual_models[member]
        for key, value in expected.items():
            checks.check(name + "." + member + "." + key, record[key] == value,
                         actual=record[key], expected=value)
        checks.check(name + "." + member + ".formation", record["formationIntact"] in
                     (("true", "1") if intact else ("false", "0")), actual=record["formationIntact"])
        checks.near(name + "." + member + ".fixed_cd", float(record["cd"]), CD[member])
        result[member] = record
    return result


def runtime(checks, args, net, mode, late=False, legacy=False, wrong_leader=False):
    import traci
    name = mode.lower() + ("_late" if late else "_legacy_wrong" if wrong_leader else "_legacy" if legacy else "_cutin")
    directory = args.output_dir / name
    directory.mkdir(parents=True, exist_ok=True)
    routes(directory / "routes.xml", mode, late=late, legacy=legacy, wrong_leader=wrong_leader)
    argv = command(args, net, directory)
    connection = None
    snapshots = {}
    try:
        with (directory / "stdout.txt").open("w", encoding="utf-8") as stdout:
            traci.start(argv, label=name, stdout=stdout, numRetries=3)
            connection = traci.getConnection(name)
            for unused in range(5):
                connection.simulationStep()
            for vehicle_id in connection.vehicle.getIDList():
                connection.vehicle.setLaneChangeMode(vehicle_id, 0)
            if legacy:
                for vehicle_id in ("B", "C"):
                    actual = connection.vehicle.getParameter(vehicle_id, "device.rtsim.activeController")
                    expected = "ACC" if wrong_leader else mode
                    checks.check(name + "." + vehicle_id + ".mode", actual == expected,
                                 actual=actual, expected=expected)
                    leader = connection.vehicle.getParameter(vehicle_id, "device.rtsim.assignedLeader")
                    checks.check(name + "." + vehicle_id + ".leader", leader == ("X" if wrong_leader else "A"))
            elif late:
                snapshots["pending"] = state(checks, connection, name + ".pending", mode, False, ("B", "C"))
                while connection.simulation.getTime() < 2.5:
                    connection.simulationStep()
                snapshots["joined"] = state(checks, connection, name + ".joined", mode, True)
                connection.vehicle.remove("A")
                for unused in range(3):
                    connection.simulationStep()
                snapshots["leader_exited"] = state(checks, connection, name + ".leader_exited", mode, False, ("B", "C"))
            else:
                snapshots["before"] = state(checks, connection, name + ".before", mode, True)
                between = (connection.vehicle.getLanePosition("B") + connection.vehicle.getLanePosition("C")) / 2
                connection.vehicle.moveTo("X", "road_0", between)
                for unused in range(3):
                    connection.simulationStep()
                checks.check(name + ".outsider_ahead", connection.vehicle.getLeader("C", 300)[0] == "X")
                snapshots["cutin"] = state(checks, connection, name + ".cutin", mode, False)
                checks.check(name + ".outsider_not_admitted",
                             connection.vehicle.getParameter("X", "device.rtsim.platoonID") == "")
                connection.vehicle.moveTo("X", "road_1", connection.vehicle.getLanePosition("X"))
                for unused in range(3):
                    connection.simulationStep()
                snapshots["cleared"] = state(checks, connection, name + ".cleared", mode, True)
                # A reload in the same process must discard the previous identity registry.
                reloaded = directory / "reload"
                reloaded.mkdir(exist_ok=True)
                routes(reloaded / "routes.xml", mode, group="P2")
                reload_argv = command(args, net, reloaded)
                connection.load(reload_argv[1:])
                for unused in range(5):
                    connection.simulationStep()
                snapshots["reload"] = state(checks, connection, name + ".reload", mode, True, group="P2")
            connection.close()
            connection = None
    finally:
        if connection is not None:
            connection.close()
    (directory / "states.json").write_text(json.dumps(snapshots, indent=2) + "\n", encoding="utf-8")
    collisions = ET.parse(directory / "collisions.xml").getroot().findall("collision")
    checks.check(name + ".no_collisions", not collisions)
    checks.commands.append({"name": name, "argv": argv, "returncode": 0})


def rejected(checks, args, net, name, overrides, diagnostic, missing_device=None):
    directory = args.output_dir / name
    directory.mkdir(parents=True, exist_ok=True)
    routes(directory / "routes.xml", overrides=overrides, missing_device=missing_device)
    result = checks.command(name, command(args, net, directory), directory)
    checks.check(name + ".rejected", result.returncode != 0, returncode=result.returncode)
    checks.check(name + ".diagnostic", diagnostic in result.stderr,
                 expected=diagnostic, stderr=result.stderr[-3000:])


def krauss(checks, args, net, inherited=False, native=False):
    name = "krauss_native_interval" if native else "krauss_device_interval_" + ("vtype" if inherited else "vehicle")
    directory = args.output_dir / name
    directory.mkdir(parents=True, exist_ok=True)
    root = ET.Element("routes")
    kind = ET.SubElement(root, "vType", id="k", carFollowModel="Krauss", emissionClass=common.EMISSION_CLASS,
                         sigmaStep="0.3", sigma="0.4")
    ET.SubElement(root, "route", id="r", edges="road")
    vehicle = ET.SubElement(root, "vehicle", id="K", type="k", route="r", depart="0")
    common.add_params(vehicle, {"has.rtsim.device": "true", "device.rtsim.automation-level": 3})
    if not native:
        common.add_params(kind if inherited else vehicle, {"device.rtsim.sigma-step": 0.3})
    common.write_xml(directory / "routes.xml", root)
    result = checks.command(name, command(args, net, directory), directory)
    if native:
        checks.check(name + ".accepted", result.returncode == 0, stderr=result.stderr)
    else:
        checks.check(name + ".rejected", result.returncode != 0)
        checks.check(name + ".instruction", "use the native vType sigmaStep attribute" in result.stderr,
                     stderr=result.stderr[-2000:])


def mixed_models(checks, args, net, krauss_members, reject=False):
    import traci
    name = "mixed_krauss_" + "".join(krauss_members)
    directory = args.output_dir / name
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / "routes.xml"
    routes(path)
    root = ET.parse(path).getroot()
    attributes = dict(root.find("vType").attrib)
    attributes.update({"id": "krauss", "carFollowModel": "Krauss"})
    del attributes["lanesCount"]
    root.insert(1, ET.Element("vType", attributes))
    for member in krauss_members:
        vehicle = root.find("./vehicle[@id='%s']" % member)
        vehicle.set("type", "krauss")
        vehicle.find("./param[@key='device.rtsim.controller']").set("value", "unchanged")
    common.write_xml(path, root)
    argv = command(args, net, directory)
    if reject:
        result = checks.command(name, argv, directory)
        checks.check(name + ".rejected", result.returncode != 0)
        checks.check(name + ".peer_diagnostic", "must use carFollowModel=CC" in result.stderr,
                     stderr=result.stderr[-2000:])
        return
    connection = None
    try:
        with (directory / "stdout.txt").open("w", encoding="utf-8") as stdout:
            traci.start(argv, label=name, stdout=stdout, numRetries=3)
            connection = traci.getConnection(name)
            for unused in range(5):
                connection.simulationStep()
            snapshot = state(checks, connection, name, "PLOEG", True,
                             manual_models={member: "Krauss" for member in krauss_members})
            (directory / "states.json").write_text(json.dumps(snapshot, indent=2) + "\n", encoding="utf-8")
            connection.close()
            connection = None
    finally:
        if connection is not None:
            connection.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sumo", required=True, type=Path)
    parser.add_argument("--netconvert", required=True, type=Path)
    parser.add_argument("--phemlight-path", required=True, type=Path)
    parser.add_argument("--output-dir", type=Path, default=Path(__file__).resolve().parent / "generated" / "platoon_identity")
    args = parser.parse_args()
    for key in ("sumo", "netconvert", "phemlight_path", "output_dir"):
        setattr(args, key, getattr(args, key).resolve())
    args.output_dir.mkdir(parents=True, exist_ok=True)
    common.GENERATED = args.output_dir
    sys.path.insert(0, str(common.ROOT / "tools"))
    checks = common.Checks()
    net = network(checks, args)
    for mode in ("PLOEG", "CACC"):
        for late, legacy, wrong in ((False, False, False), (True, False, False),
                                    (False, True, False), (False, True, True)):
            checks.group(mode + str((late, legacy, wrong)),
                         lambda m=mode, a=late, b=legacy, c=wrong: runtime(checks, args, net, m, a, b, c))
    cases = [
        ("duplicate", {"A": {"device.rtsim.members": "A B B C"}}, "duplicate vehicle"),
        ("missing_self", {"A": {"device.rtsim.members": "B C"}}, "must include this vehicle"),
        ("missing_id", {"A": {"device.rtsim.platoon-id": None}}, "must be specified together"),
        ("conflicting_order", {"B": {"device.rtsim.members": "B A C"}}, "ordered consistently"),
        ("two_groups", {"B": {"device.rtsim.platoon-id": "P2"}}, "already assigned"),
        ("wrong_leader", {"B": {"device.rtsim.leader": "X"}}, "conflicts with ordered members"),
        ("wrong_front", {"C": {"device.rtsim.front": "A"}}, "conflicts with ordered members"),
        ("wrong_position", {"C": {"device.rtsim.position": 2}}, "conflicts with ordered members"),
        ("wrong_size", {"C": {"device.rtsim.platoon-size": 4}}, "conflicts with ordered members"),
        ("unregistered_member", {"B": {"device.rtsim.platoon-id": None, "device.rtsim.members": None,
                                       "device.rtsim.controller": "ACC", "device.rtsim.model": None,
                                       "device.rtsim.cd": 0.45}}, "must declare the same platoon-id"),
    ]
    for name, overrides, diagnostic in cases:
        checks.group(name, lambda n=name, o=overrides, d=diagnostic: rejected(checks, args, net, n, o, d))
    checks.group("unequipped_member", lambda: rejected(checks, args, net, "unequipped_member", {},
                 "must declare the same platoon-id", missing_device="B"))
    for inherited, native in ((False, False), (True, False), (False, True)):
        checks.group("krauss" + str((inherited, native)),
                     lambda i=inherited, n=native: krauss(checks, args, net, i, n))
    for members, reject in ((("C",), False), (("A", "B", "C"), False), (("A",), True), (("B",), True)):
        checks.group("mixed" + str(members), lambda m=members, r=reject: mixed_models(checks, args, net, m, r))
    report = {"checks": checks.results, "commands": checks.commands,
              "passed": sum(item["passed"] for item in checks.results),
              "failed": sum(not item["passed"] for item in checks.results)}
    (args.output_dir / "platoon_identity_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print("%d passed; %d failed" % (report["passed"], report["failed"]))
    return bool(report["failed"])


if __name__ == "__main__":
    raise SystemExit(main())
