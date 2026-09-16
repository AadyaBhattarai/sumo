#!/usr/bin/env python3
"""Regression of vehicle-local road loads against copied RTSim heavy-truck data.

Only the selected three original data files are read. Generated copies receive
SUMO's current HDV_ prefix and .PHEMLight.veh filename suffix and optional Cd/Fr0 changes. The comparison
proves mechanical equivalence to those data edits, not empirical calibration,
CFD accuracy, or the physical correctness of upstream grade conventions.
"""
# Copyright (C) 2026 RTSIm contributors.
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later
import argparse
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import traceback
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
GENERATED = Path(__file__).resolve().parent / 'generated_legacy'
CLASS = 'HDV_RT_II_D_EU0'
SOURCE_STEM = 'RT_II_D_EU0'
SUFFIXES = ('.veh', '.csv', '_FC.csv')
FIELDS = ('CO2', 'CO', 'HC', 'NOx', 'PMx', 'fuel', 'electricity', 'speed')


class Checks:
    def __init__(self):
        self.results = []
        self.commands = []

    def check(self, name, passed, **details):
        self.results.append(dict(name=name, passed=bool(passed), **details))
        if not passed:
            raise AssertionError(name + ': ' + str(details))

    def command(self, name, argv, directory):
        directory.mkdir(parents=True, exist_ok=True)
        try:
            result = subprocess.run([str(arg) for arg in argv], cwd=directory,
                                    capture_output=True, text=True, errors='replace', timeout=120)
        except subprocess.TimeoutExpired as error:
            self.commands.append(dict(name=name, argv=[str(x) for x in argv],
                                      cwd=str(directory), timed_out_seconds=120))
            for suffix, content in (('stdout', error.stdout), ('stderr', error.stderr)):
                if isinstance(content, bytes):
                    content = content.decode('utf-8', errors='replace')
                (directory / (name + '.' + suffix + '.txt')).write_text(content or '', encoding='utf-8')
            raise
        (directory / (name + '.stdout.txt')).write_text(result.stdout, encoding='utf-8')
        (directory / (name + '.stderr.txt')).write_text(result.stderr, encoding='utf-8')
        self.commands.append(dict(name=name, argv=[str(x) for x in argv],
                                  cwd=str(directory), returncode=result.returncode))
        self.check(name + '.success', result.returncode == 0,
                   returncode=result.returncode, stderr=result.stderr[-2500:])
        return result

    def group(self, name, function):
        (GENERATED / (name + '.exception.txt')).unlink(missing_ok=True)
        try:
            function()
        except Exception as error:
            if not self.results or self.results[-1]['passed']:
                self.results.append(dict(name=name, passed=False, error=str(error)))
            (GENERATED / (name + '.exception.txt')).write_text(traceback.format_exc(), encoding='utf-8')
            print('FAIL:', name, str(error))
        else:
            print('PASS:', name)


def xml(path, root):
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def dataset(checks, source, name, values=None):
    directory = GENERATED / 'datasets' / name
    directory.mkdir(parents=True, exist_ok=True)
    for suffix in SUFFIXES:
        shutil.copyfile(source / (SOURCE_STEM + suffix), directory / (CLASS + ('.PHEMLight.veh' if suffix == '.veh' else suffix)))
    if values:
        path = directory / (CLASS + '.PHEMLight.veh')
        lines = path.read_text(encoding='utf-8').splitlines(keepends=True)
        for key, marker in (('cd', 'c Cd value [-]'), ('fr0', 'c Fr0')):
            if key in values:
                positions = [i for i, line in enumerate(lines) if line.strip() == marker]
                checks.check(name + '.unique_' + key, len(positions) == 1, matches=len(positions))
                lines[positions[0] + 1] = format(values[key], '.12g') + '\n'
        path.write_text(''.join(lines), encoding='utf-8')
    else:
        for suffix in SUFFIXES:
            checks.check(name + '.byte_identical' + suffix,
                         sha256(source / (SOURCE_STEM + suffix)) == sha256(directory / (CLASS + ('.PHEMLight.veh' if suffix == '.veh' else suffix))))
    return directory


def network(checks, args, name, grade):
    directory = GENERATED / name
    directory.mkdir(parents=True, exist_ok=True)
    nodes, edges = ET.Element('nodes'), ET.Element('edges')
    for lane in range(2):
        for i in range(4):
            ET.SubElement(nodes, 'node', id=f'n{lane}_{i}', x=str(500 * i),
                          y=str(50 * lane), z=str(500 * i * grade), type='priority')
        for i, speed in enumerate((25, 6, 25)):
            ET.SubElement(edges, 'edge', id=f'e{lane}_{i}', **{
                'from': f'n{lane}_{i}', 'to': f'n{lane}_{i + 1}',
                'numLanes': '1', 'speed': str(speed), 'priority': '1'})
    xml(directory / 'nodes.nod.xml', nodes)
    xml(directory / 'edges.edg.xml', edges)
    checks.command(name + '_netconvert', [args.netconvert, '--node-files', 'nodes.nod.xml',
                   '--edge-files', 'edges.edg.xml', '--output-file', 'network.net.xml',
                   '--no-internal-links', 'true', '--precision', '10'], directory)
    return directory / 'network.net.xml'


def rows(path):
    return {(float(step.get('time')), v.get('id')): dict(v.attrib)
            for step in ET.parse(path).getroot().findall('timestep') for v in step.findall('vehicle')}


def simulate(checks, args, name, net, data, parameters=None):
    """Two noninteracting trucks share one vType and one cached emission class."""
    directory = GENERATED / name
    directory.mkdir(parents=True, exist_ok=True)
    route = ET.Element('routes')
    ET.SubElement(route, 'vType', id='shared', vClass='truck', carFollowModel='Krauss',
                  emissionClass='PHEMlight/' + CLASS, sigma='0', tau='1', length='12',
                  accel='1', decel='4.5', emergencyDecel='9', minGap='2.5', maxSpeed='25',
                  speedFactor='1', speedDev='0')
    for i in range(2):
        ET.SubElement(route, 'route', id=f'r{i}', edges=' '.join(f'e{i}_{j}' for j in range(3)))
    for i in range(2):
        vehicle = ET.SubElement(route, 'vehicle', id=f'v{i}', type='shared', route=f'r{i}',
                                depart='0', departLane='0', departPos='0', departSpeed='0')
        if parameters is not None:
            ET.SubElement(vehicle, 'param', key='has.rtsim.device', value='true')
            for key, value in parameters[i].items():
                ET.SubElement(vehicle, 'param', key='device.rtsim.' + key, value=str(value))
    xml(directory / 'routes.rou.xml', route)
    checks.command(name, [args.sumo, '--net-file', net, '--route-files', 'routes.rou.xml',
                   '--phemlight-path', data, '--seed', '42', '--step-length', '.1', '--end', '400',
                   '--no-step-log', 'true', '--duration-log.disable', 'true', '--precision', '10',
                   '--device.emissions.probability', '1', '--emission-output', 'emissions.xml',
                   '--emission-output.precision', '10', '--emission-output.step-scaled', 'false',
                   '--fcd-output', 'fcd.xml', '--fcd-output.acceleration', 'true',
                   '--tripinfo-output', 'tripinfo.xml'], directory)
    trips = {v.get('id'): v for v in ET.parse(directory / 'tripinfo.xml').getroot().findall('tripinfo')}
    checks.check(name + '.all_arrived', set(trips) == {'v0', 'v1'}, actual=sorted(trips))
    fcd, emission = rows(directory / 'fcd.xml'), rows(directory / 'emissions.xml')
    checks.check(name + '.acceleration_exercised', any(float(v['acceleration']) > .1 for v in fcd.values()))
    checks.check(name + '.deceleration_exercised', any(float(v['acceleration']) < -.1 for v in fcd.values()))
    for vehicle_id, trip in trips.items():
        checks.check(name + '.positive_fuel.' + vehicle_id,
                     float(trip.find('emissions').get('fuel_abs')) > 0)
        checks.check(name + '.device_output.' + vehicle_id,
                     (trip.find('rtsim') is not None) == (parameters is not None))
    return dict(fcd=fcd, emissions=emission, trips=trips)


def near_values(checks, name, left, right):
    checks.check(name, len(left) == len(right) and bool(left) and all(
        math.isfinite(a) and math.isfinite(b) and math.isclose(a, b, rel_tol=1e-8, abs_tol=1e-7)
        for a, b in zip(left, right)), samples=len(left),
        max_absolute_difference=max((abs(a - b) for a, b in zip(left, right)), default=None))


def compare(checks, name, left, right, vehicles=('v0', 'v1')):
    for category, fields in (('fcd', ('x', 'y', 'speed', 'pos', 'acceleration')),
                             ('emissions', FIELDS)):
        a = {k: v for k, v in left[category].items() if k[1] in vehicles}
        b = {k: v for k, v in right[category].items() if k[1] in vehicles}
        checks.check(name + '.' + category + '.keys', bool(a) and a.keys() == b.keys(),
                     samples=len(a), control_samples=len(b))
        for field in fields:
            near_values(checks, name + '.' + category + '.' + field,
                        [float(a[k][field]) for k in a], [float(b[k][field]) for k in a])
    for vehicle in vehicles:
        a, b = left['trips'][vehicle], right['trips'][vehicle]
        for key in ('arrival', 'duration', 'routeLength', 'waitingTime', 'timeLoss'):
            near_values(checks, name + '.trip.' + vehicle + '.' + key,
                        [float(a.get(key))], [float(b.get(key))])
        for key, value in a.find('emissions').attrib.items():
            near_values(checks, name + '.total.' + vehicle + '.' + key,
                        [float(value)], [float(b.find('emissions').get(key))])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sumo', type=Path, required=True)
    parser.add_argument('--netconvert', type=Path, required=True)
    parser.add_argument('--source', type=Path, default=ROOT.parent / 'RTSim-main/examples/simulation/PHEMLight/Model1/2truck/5/90/Lower')
    args = parser.parse_args()
    args.sumo, args.netconvert, args.source = args.sumo.resolve(), args.netconvert.resolve(), args.source.resolve()
    GENERATED.mkdir(parents=True, exist_ok=True)
    os.environ['SUMO_HOME'] = str(ROOT)
    dlls = ROOT.parent / 'toolchain/xerces-install/bin'
    if dlls.exists():
        os.environ['PATH'] = str(dlls) + os.pathsep + os.environ.get('PATH', '')
    checks = Checks()
    source_hashes = {}
    try:
        source_hashes = {suffix: sha256(args.source / (SOURCE_STEM + suffix)) for suffix in SUFFIXES}
        base = dataset(checks, args.source, 'original')
        values = {'cd_only': {'cd': .4}, 'fr0_only': {'fr0': .02},
                  'combined': {'cd': .4, 'fr0': .02}, 'other_vehicle': {'cd': .9, 'fr0': .005}}
        datasets = {name: dataset(checks, args.source, name, params) for name, params in values.items()}
        for name, grade in (('flat', 0), ('uphill', .05), ('downhill', -.05)):
            def exercise(name=name, grade=grade):
                net = network(checks, args, name, grade)
                baseline = simulate(checks, args, name + '_disabled', net, base)
                enabled = simulate(checks, args, name + '_no_overrides', net, base, ({}, {}))
                compare(checks, name + '.no_override_equivalence', enabled, baseline)
                controls = {}
                for variant in ('cd_only', 'fr0_only', 'combined', 'other_vehicle'):
                    params = values[variant]
                    controls[variant] = simulate(checks, args, name + '_' + variant + '_data', net, datasets[variant])
                    native = simulate(checks, args, name + '_' + variant + '_device', net, base, (params, params))
                    compare(checks, name + '.' + variant + '.equivalence', native, controls[variant])
                mixed = simulate(checks, args, name + '_mixed', net, base,
                                 (values['combined'], values['other_vehicle']))
                compare(checks, name + '.isolation_v0', mixed, controls['combined'], ('v0',))
                compare(checks, name + '.isolation_v1', mixed, controls['other_vehicle'], ('v1',))
                checks.check(name + '.override_changes_fuel',
                    not math.isclose(float(baseline['trips']['v0'].find('emissions').get('fuel_abs')),
                                     float(controls['combined']['trips']['v0'].find('emissions').get('fuel_abs')), rel_tol=1e-5))
            checks.group(name, exercise)
        checks.check('source_files_unchanged', source_hashes == {
            suffix: sha256(args.source / (SOURCE_STEM + suffix)) for suffix in SUFFIXES})
    except Exception as error:
        checks.results.append(dict(name='setup', passed=False, error=str(error)))
        (GENERATED / 'setup.exception.txt').write_text(traceback.format_exc(), encoding='utf-8')
        print('FAIL: setup', error)
    report = dict(created=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  purpose='Mechanical equivalence against copied RTSim heavy-truck data; not scientific calibration.',
                  filename_adapter='RT_II_D_EU0 -> HDV_RT_II_D_EU0 class prefix; .veh -> .PHEMLight.veh vehicle suffix for current SUMO. Data bodies preserved except explicitly tested Cd/Fr0 edits.',
                  source=str(args.source), source_sha256=source_hashes,
                  passed=bool(checks.results) and all(item['passed'] for item in checks.results),
                  checks=checks.results, commands=checks.commands)
    (GENERATED / 'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(f"{sum(c['passed'] for c in checks.results)}/{len(checks.results)} checks passed; report: {GENERATED / 'report.json'}")
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
