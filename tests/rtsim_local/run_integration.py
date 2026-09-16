#!/usr/bin/env python3
"""Mechanical regression checks for the private native RTSIm prototype.

This uses SUMO's bundled PHEMlight5 passenger-car data and synthetic roads.
It checks parameter plumbing and isolation, not validation of truck research,
CFD measurements, dynamic platoon membership, or measured-gap aerodynamics.
All generated inputs, outputs, logs, and the JSON report stay in generated/.
"""
# Eclipse SUMO, Simulation of Urban MObility; https://eclipse.dev/sumo
# Copyright (C) 2026 RTSIm contributors.
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later

import argparse
import datetime
import json
import math
from pathlib import Path
import subprocess
import sys
import traceback
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[2]
GENERATED = Path(__file__).resolve().parent / "generated"
EMISSION_CLASS = "PHEMlight5/PC_EU4_D_MW"
STEP = 0.1


class Checks:
    def __init__(self):
        self.results = []
        self.commands = []

    def check(self, name, passed, **details):
        self.results.append({"name": name, "passed": bool(passed), **details})
        if not passed:
            raise AssertionError(name + ": " + str(details))

    def near(self, name, actual, expected, tolerance=1e-7):
        self.check(name, math.isfinite(actual) and math.isclose(
            actual, expected, rel_tol=tolerance, abs_tol=tolerance),
            actual=actual, expected=expected, tolerance=tolerance)

    def group(self, name, function):
        start = len(self.results)
        try:
            function()
        except Exception as error:
            if len(self.results) == start or self.results[-1]["passed"]:
                self.results.append({"name": name, "passed": False,
                                     "error": str(error)})
            print("FAIL:", name, error)
            (GENERATED / (name + ".exception.txt")).write_text(
                traceback.format_exc(), encoding="utf-8")
        else:
            print("PASS:", name)

    def command(self, name, command, directory):
        directory.mkdir(parents=True, exist_ok=True)
        result = subprocess.run([str(item) for item in command], cwd=directory,
                                capture_output=True, text=True, timeout=120,
                                errors="replace")
        (directory / (name + ".stdout.txt")).write_text(result.stdout, encoding="utf-8")
        (directory / (name + ".stderr.txt")).write_text(result.stderr, encoding="utf-8")
        self.commands.append({"name": name, "argv": [str(item) for item in command],
                              "cwd": str(directory), "returncode": result.returncode})
        return result


def write_xml(path, root):
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def add_params(element, values):
    for key, value in values.items():
        ET.SubElement(element, "param", key=key, value=str(value))


def make_network(checks, args, name, grade=0.0, road_types=None):
    directory = GENERATED / name
    directory.mkdir(parents=True, exist_ok=True)
    nodes = ET.Element("nodes")
    for index in range(3):
        ET.SubElement(nodes, "node", id="n" + str(index), x=str(500 * index),
                      y="0", z=str(500 * index * grade), type="priority")
    edges = ET.Element("edges")
    for index in range(2):
        edge = ET.SubElement(edges, "edge", {"id": "e" + str(index),
                             "from": "n" + str(index), "to": "n" + str(index + 1),
                             "numLanes": "1", "speed": "20", "priority": "1"})
        if road_types:
            add_params(edge, {"rtsim.road-type": road_types[index]})
    write_xml(directory / "nodes.nod.xml", nodes)
    write_xml(directory / "edges.edg.xml", edges)
    network = directory / "network.net.xml"
    result = checks.command("netconvert", [args.netconvert, "--node-files", "nodes.nod.xml",
                            "--edge-files", "edges.edg.xml", "--output-file", network,
                            "--no-internal-links", "true", "--precision", "10"], directory)
    checks.check(name + ".netconvert", result.returncode == 0, stderr=result.stderr)
    if road_types:
        root = ET.parse(network).getroot()
        for index, road in enumerate(road_types):
            found = root.find("./edge[@id='e%d']/param[@key='rtsim.road-type']" % index)
            checks.check(name + ".edge_parameter_%d" % index,
                         found is not None and found.get("value") == road)
    return network


def route_file(path, vehicles, native, car_follow="Krauss"):
    """Native vehicles share a base type; direct controls get distinct types."""
    root = ET.Element("routes")
    common = dict(carFollowModel=car_follow, emissionClass=EMISSION_CLASS,
                  accel="1", decel="4.5", emergencyDecel="9", length="5",
                  minGap="2.5", maxSpeed="20", speedFactor="1", speedDev="0")
    if native:
        ET.SubElement(root, "vType", id="base", sigma="0.5", tau="1", **common)
    else:
        for vehicle in vehicles:
            settings = vehicle["settings"]
            vtype = ET.SubElement(root, "vType", id="direct_" + vehicle["id"],
                                 sigma=str(settings["sigma"]), tau=str(settings["tau"]), **common)
            add_params(vtype, {"airDragCoefficient": settings["cd"],
                               "rollDragCoefficient": settings["fr0"]})
    ET.SubElement(root, "route", id="r", edges="e0 e1")
    for vehicle in vehicles:
        element = ET.SubElement(root, "vehicle", id=vehicle["id"],
                                type="base" if native else "direct_" + vehicle["id"],
                                route="r", depart="0", departLane="0",
                                departPos=str(vehicle.get("pos", 0)),
                                departSpeed=str(vehicle.get("speed", 0)))
        if native:
            add_params(element, {"has.rtsim.device": "true"})
            add_params(element, {"device.rtsim." + key: value
                                 for key, value in vehicle["settings"].items()})
    write_xml(path, root)


def simulate(checks, args, name, network, vehicles, native=True,
             car_follow="Krauss", extra=(), reject=None):
    directory = GENERATED / name
    directory.mkdir(parents=True, exist_ok=True)
    route_file(directory / "routes.rou.xml", vehicles, native, car_follow)
    command = [args.sumo, "--net-file", network, "--route-files", "routes.rou.xml",
               "--phemlight-path", args.phemlight_path, "--step-length", str(STEP),
               "--seed", "42", "--end", "200", "--no-step-log", "true",
               "--duration-log.disable", "true", "--precision", "10",
               "--device.emissions.probability", "1",
               "--emission-output", "emissions.xml", "--emission-output.precision", "10",
               "--emission-output.step-scaled", "false", "--fcd-output", "fcd.xml",
               "--fcd-output.acceleration", "true", "--tripinfo-output", "tripinfo.xml"]
    result = checks.command(name, command + list(extra), directory)
    if reject is not None:
        diagnostic = result.stdout + result.stderr
        checks.check(name + ".rejected", result.returncode != 0,
                     returncode=result.returncode)
        checks.check(name + ".diagnostic", "RTSIm" in diagnostic and reject in diagnostic,
                     expected=reject, diagnostic=diagnostic[-2500:])
        return None
    checks.check(name + ".run", result.returncode == 0, stderr=result.stderr)
    trips = {item.get("id"): item for item in ET.parse(directory / "tripinfo.xml").getroot().findall("tripinfo")}
    checks.check(name + ".all_arrived", set(trips) == {v["id"] for v in vehicles},
                 actual=sorted(trips))
    for vehicle_id, trip in trips.items():
        emission = trip.find("emissions")
        checks.check(name + ".fuel_recorded." + vehicle_id,
                     emission is not None and float(emission.get("fuel_abs", "0")) > 0)
        checks.check(name + ".device_output." + vehicle_id,
                     (trip.find("rtsim") is not None) == native)
        if native:
            checks.check(name + ".device_updated." + vehicle_id,
                         int(trip.find("rtsim").get("steps", "0")) > 0)
    return {"directory": directory, "trips": trips,
            "fcd": timestep_rows(directory / "fcd.xml"),
            "emissions": timestep_rows(directory / "emissions.xml")}


def timestep_rows(path):
    return {(float(step.get("time")), vehicle.get("id")): dict(vehicle.attrib)
            for step in ET.parse(path).getroot().findall("timestep")
            for vehicle in step.findall("vehicle")}


def compare_rows(checks, name, left, right, fields):
    checks.check(name + ".sample_keys", bool(left) and left.keys() == right.keys(),
                 left_samples=len(left), right_samples=len(right))
    for field in fields:
        checks.check(name + ".present." + field,
                     all(field in left[key] and field in right[key] for key in left))
        differences = [abs(float(left[key][field]) - float(right[key][field])) for key in left]
        valid = all(math.isclose(float(left[key][field]), float(right[key][field]),
                                rel_tol=1e-8, abs_tol=1e-7) for key in left)
        checks.check(name + ".equal." + field, valid, max_absolute_difference=max(differences))
    checks.check(name + ".lane_sequence", all(left[key].get("lane") == right[key].get("lane") for key in left))


def compare_runs(checks, name, native, direct):
    compare_rows(checks, name + ".fcd", native["fcd"], direct["fcd"],
                 ("x", "y", "speed", "pos", "acceleration"))
    compare_rows(checks, name + ".emissions", native["emissions"], direct["emissions"],
                 ("CO2", "CO", "HC", "NOx", "PMx", "fuel", "electricity", "speed"))
    for vehicle_id, trip in native["trips"].items():
        control = direct["trips"][vehicle_id]
        for field in ("arrival", "duration", "routeLength", "waitingTime", "timeLoss"):
            checks.near(name + ".trip." + vehicle_id + "." + field,
                        float(trip.get(field)), float(control.get(field)))
        emission = trip.find("emissions")
        for field, value in emission.attrib.items():
            checks.near(name + ".totals." + vehicle_id + "." + field,
                        float(value), float(control.find("emissions").get(field)))


def vehicle(vehicle_id="v0", **settings):
    return {"id": vehicle_id, "settings": {"sigma": 0, "tau": 0.8, "cd": 0.6, "fr0": 0.01, **settings}}


def with_settings(original, **changes):
    return {**original, "settings": {**original["settings"], **changes}}


def run_suite(checks, args):
    networks = {}

    def networks_group():
        networks["flat"] = make_network(checks, args, "network_flat")
        networks["grade"] = make_network(checks, args, "network_grade", grade=0.05)
        networks["transition"] = make_network(checks, args, "network_transition", grade=0.05,
                                               road_types=("primary", "cross_country"))
    checks.group("networks", networks_group)

    def single_group():
        native = simulate(checks, args, "single_native", networks["flat"], [vehicle()])
        direct = simulate(checks, args, "single_direct", networks["flat"], [vehicle()], native=False)
        compare_runs(checks, "single", native, direct)
        state = native["trips"]["v0"].find("rtsim")
        for key, value in vehicle()["settings"].items():
            checks.near("single.state." + key, float(state.get(key)), value)
    checks.group("single", single_group)

    def isolation_group():
        first = with_settings(vehicle(), tau=0.7, cd=0.4, fr0=0.006923)
        first.update(pos=50, speed=0)
        second = with_settings(vehicle("v1"), tau=1.5, cd=0.9, fr0=0.025)
        second.update(pos=0, speed=10)
        native = simulate(checks, args, "isolation_native", networks["flat"], [first, second])
        direct = simulate(checks, args, "isolation_direct", networks["flat"], [first, second], native=False)
        compare_runs(checks, "isolation", native, direct)
        for item in (first, second):
            state = native["trips"][item["id"]].find("rtsim")
            for key, expected in item["settings"].items():
                checks.near("isolation.state." + item["id"] + "." + key, float(state.get(key)), expected)
    checks.group("isolation", isolation_group)

    cfd = GENERATED / "coefficients.csv"
    cfd.write_text("model_id,platoon_size,gap_m,position,cd_lower,cd_upper\n"
                   "Synthetic,2,5,2,0.5,0.7\nSynthetic,2,15,2,0.7,0.9\n", encoding="utf-8")

    def cfd_group():
        item = vehicle()
        del item["settings"]["cd"]
        item["settings"].update({"model": "Synthetic", "platoon-size": 2,
                                 "position": 2, "gap": 10, "cd-bound": "lower"})
        native = simulate(checks, args, "cfd_native", networks["flat"], [item],
                          extra=("--device.rtsim.cfd-file", cfd))
        direct = simulate(checks, args, "cfd_direct", networks["flat"], [vehicle()], native=False)
        compare_runs(checks, "cfd", native, direct)
        state = native["trips"]["v0"].find("rtsim")
        for key, expected in {"cd": 0.6, "position": 2, "platoonSize": 2, "nominalGap": 10}.items():
            checks.near("cfd.state." + key, float(state.get(key)), expected)
    checks.group("cfd", cfd_group)

    def profile_group():
        item = vehicle()
        del item["settings"]["sigma"]
        del item["settings"]["tau"]
        item["settings"]["automation-level"] = 4
        native = simulate(checks, args, "profile_native", networks["flat"], [item])
        direct = simulate(checks, args, "profile_direct", networks["flat"],
                          [with_settings(vehicle(), sigma=0, tau=0.7)], native=False)
        compare_runs(checks, "profile", native, direct)
        state = native["trips"]["v0"].find("rtsim")
        checks.near("profile.level", float(state.get("automationLevel")), 4)
    checks.group("profile", profile_group)

    def road_grade_group():
        item = with_settings(vehicle(), fr0=0.006923)
        item["speed"] = 20
        flat = simulate(checks, args, "road_flat", networks["flat"], [item])
        grade = simulate(checks, args, "road_grade", networks["grade"], [item])
        direct = simulate(checks, args, "road_grade_direct", networks["grade"], [item], native=False)
        compare_runs(checks, "grade", grade, direct)
        flat_fuel = float(flat["trips"]["v0"].find("emissions").get("fuel_abs"))
        grade_fuel = float(grade["trips"]["v0"].find("emissions").get("fuel_abs"))
        checks.check("grade.increases_fuel", grade_fuel > flat_fuel,
                     flat_fuel=flat_fuel, grade_fuel=grade_fuel)
        transition = simulate(checks, args, "road_transition", networks["transition"], [item])
        state = transition["trips"]["v0"].find("rtsim")
        checks.near("transition.final_fr0", float(state.get("fr0")), 0.025)
        checks.near("transition.final_slope", float(state.get("lastSlopeDegrees")),
                    math.degrees(math.atan(0.05)), tolerance=1e-5)
        means = []
        for edge in ("e0_0", "e1_0"):
            values = [float(row["fuel"]) for row in transition["emissions"].values()
                      if row.get("lane") == edge and 100 < float(row["pos"]) < 400
                      and float(row["speed"]) > 19.99]
            checks.check("transition.steady_samples." + edge, len(values) > 50,
                         samples=len(values))
            means.append(sum(values) / len(values))
        checks.check("transition.increases_fuel", means[1] > means[0],
                     primary_mean_fuel=means[0], cross_country_mean_fuel=means[1])
    checks.group("road_grade", road_grade_group)

    def paper_profiles_group():
        # RTSim manuscript, SSRN 6353058, Table 2, page 6.
        pairs = [(0.5, 1.0), (0.4, 0.95), (0.3, 0.90), (0.2, 0.80), (0.0, 0.70), (0.0, 0.60)]
        for level, (sigma, tau) in enumerate(pairs):
            item = vehicle()
            del item["settings"]["sigma"]
            del item["settings"]["tau"]
            item["settings"]["automation-level"] = level
            result = simulate(checks, args, "paper_profile_%d" % level, networks["flat"], [item])
            state = result["trips"]["v0"].find("rtsim")
            checks.near("paper_profile_%d.sigma" % level, float(state.get("sigma")), sigma)
            checks.near("paper_profile_%d.tau" % level, float(state.get("tau")), tau)
    checks.group("paper_profiles", paper_profiles_group)

    def state_guard_group():
        state_file = GENERATED / "baseline_state.xml"
        simulate(checks, args, "state_save_rejected", networks["flat"], [vehicle()],
                 extra=("--save-state.times", "1", "--save-state.files", state_file),
                 reject="state saving is not implemented")
        simulate(checks, args, "state_baseline", networks["flat"], [vehicle()], native=False,
                 extra=("--save-state.times", "1", "--save-state.files", state_file))
        # SUMO restores only the device IDs recorded in the snapshot.
        # Declare an unsupported RTSIm state explicitly to exercise its guard.
        snapshot = ET.parse(state_file)
        ET.SubElement(snapshot.getroot().find("vehicle"), "device", id="rtsim_v0", state="")
        snapshot.write(state_file, encoding="utf-8", xml_declaration=True)
        directory = GENERATED / "state_load_rejected"
        result = checks.command("state_load_rejected", [args.sumo, "--net-file", networks["flat"],
                                "--load-state", state_file, "--device.rtsim.probability", "1",
                                "--phemlight-path", args.phemlight_path, "--step-length", str(STEP),
                                "--end", "2", "--no-step-log", "true"], directory)
        checks.check("state_load_rejected.returncode", result.returncode != 0,
                     returncode=result.returncode)
        checks.check("state_load_rejected.diagnostic",
                     "RTSIm state restoration is not implemented" in result.stderr,
                     stderr=result.stderr)
    checks.group("state_guards", state_guard_group)

    negatives = [
        ("sigma_negative", {"sigma": -0.1}, "sigma", "Krauss"),
        ("sigma_negative_sentinel", {"sigma": -1}, "sigma", "Krauss"),
        ("sigma_too_large", {"sigma": 1.1}, "sigma", "Krauss"),
        ("tau_negative", {"tau": -0.2}, "tau", "Krauss"),
        ("tau_negative_sentinel", {"tau": -1}, "tau", "Krauss"),
        ("tau_zero", {"tau": 0}, "tau", "Krauss"),
        ("invalid_controller", {"controller": "BAD"}, "controller", "Krauss"),
        ("acc_on_krauss", {"controller": "ACC"}, "requires carFollowModel=CC", "Krauss"),
        ("unsupported_cf", {}, "automation requires Krauss", "IDM"),
        ("negative_fr0", {"fr0": -0.1}, "fr0", "Krauss"),
        ("zero_cd", {"cd": 0}, "cd", "Krauss"),
    ]
    for name, changes, message, cf in negatives:
        checks.group(name, lambda n=name, c=changes, m=message, model=cf: simulate(
            checks, args, n, networks["flat"], [with_settings(vehicle(), **c)],
            car_follow=model, reject=m))

    def unknown_cfd_group():
        item = vehicle()
        del item["settings"]["cd"]
        item["settings"]["model"] = "MissingModel"
        simulate(checks, args, "unknown_cfd_model", networks["flat"], [item],
                 extra=("--device.rtsim.cfd-file", cfd), reject="no data")
    checks.group("unknown_cfd_model", unknown_cfd_group)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sumo", type=Path, required=True)
    parser.add_argument("--netconvert", type=Path, required=True)
    parser.add_argument("--phemlight-path", type=Path,
                        default=ROOT / "data" / "emissions" / "PHEMlight5")
    args = parser.parse_args()
    args.sumo = args.sumo.resolve()
    args.netconvert = args.netconvert.resolve()
    args.phemlight_path = args.phemlight_path.resolve()
    GENERATED.mkdir(parents=True, exist_ok=True)
    checks = Checks()
    try:
        checks.check("preflight.sumo", args.sumo.is_file(), path=str(args.sumo))
        checks.check("preflight.netconvert", args.netconvert.is_file(), path=str(args.netconvert))
        checks.check("preflight.phemlight", (args.phemlight_path / "PC_EU4_D_MW.PHEMLight.veh").is_file(),
                     path=str(args.phemlight_path))
        run_suite(checks, args)
    except Exception as error:
        if not checks.results or checks.results[-1]["passed"]:
            checks.results.append({"name": "harness", "passed": False, "error": str(error)})
        print("FAIL:", error)
    failed = sum(not item["passed"] for item in checks.results)
    report = {"created_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
              "scope": "Mechanical prototype regression using synthetic roads and bundled passenger-car PHEMlight5 data; not research validation.",
              "sumo": str(args.sumo), "netconvert": str(args.netconvert),
              "phemlight_path": str(args.phemlight_path), "step_length": STEP, "seed": 42,
              "passed": len(checks.results) - failed, "failed": failed,
              "assertions": checks.results, "commands": checks.commands}
    path = GENERATED / "integration_report.json"
    path.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print("%d assertions passed; %d failed. Report: %s" % (report["passed"], failed, path))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
