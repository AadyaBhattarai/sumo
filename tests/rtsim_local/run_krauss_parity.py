#!/usr/bin/env python3
"""Differential regression for extracting SUMO's Krauss dawdling helper.

First run --mode reference using a preserved binary built before the refactor.
After rebuilding, run --mode compare against the new binary. No RTSIm device
is enabled. Trajectories, trip records, and complete saved simulation states
(including RNGs and Krauss per-vehicle variables) must match exactly after
ignoring XML formatting/comments. This is a behavior-preservation regression,
not a validation of physical driver behavior.
"""
# Copyright (C) 2026 RTSIm contributors.
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import traceback
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
GENERATED = Path(__file__).resolve().parent / 'generated_krauss_parity'
STEP = .1
SNAPSHOTS = (1, 15, 40, 100)
CASES = [dict(name=f"{'ballistic' if ballistic else 'euler'}_sigma{str(sigma).replace('.', '_')}_step{str(sigma_step).replace('.', '_')}_seed{seed}",
              ballistic=ballistic, sigma=sigma, sigma_step=sigma_step, seed=seed)
         for ballistic in (False, True) for sigma in (0., .5)
         for sigma_step in (.1, .5) for seed in (42, 73)]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def xml(path, element):
    ET.ElementTree(element).write(path, encoding='utf-8', xml_declaration=True)


def canonical(element):
    """Keep exact attributes, text and ordering; remove formatting-only space."""
    return [element.tag, sorted(element.attrib.items()), (element.text or '').strip(),
            [canonical(child) for child in element]]


class Checks:
    def __init__(self):
        self.results, self.commands = [], []

    def check(self, name, passed, **details):
        self.results.append(dict(name=name, passed=bool(passed), **details))
        if not passed:
            raise AssertionError(name + ': ' + str(details))

    def command(self, name, argv, directory):
        result = subprocess.run([str(arg) for arg in argv], cwd=directory,
                                capture_output=True, text=True, errors='replace', timeout=120)
        (directory / (name + '.stdout.txt')).write_text(result.stdout, encoding='utf-8')
        (directory / (name + '.stderr.txt')).write_text(result.stderr, encoding='utf-8')
        self.commands.append(dict(name=name, argv=[str(x) for x in argv],
                                  cwd=str(directory), returncode=result.returncode))
        self.check(name + '.success', result.returncode == 0,
                   returncode=result.returncode, stderr=result.stderr[-2000:])
        return result

    def group(self, name, action):
        (GENERATED / (name + '.exception.txt')).unlink(missing_ok=True)
        try:
            action()
        except Exception as error:
            if not self.results or self.results[-1]['passed']:
                self.results.append(dict(name=name, passed=False, error=str(error)))
            (GENERATED / (name + '.exception.txt')).write_text(traceback.format_exc(), encoding='utf-8')
            print('FAIL:', name, str(error), flush=True)
        else:
            print('PASS:', name, flush=True)


def fixtures(checks, args):
    directory = GENERATED / 'fixtures'
    directory.mkdir(parents=True, exist_ok=True)
    nodes, edges = ET.Element('nodes'), ET.Element('edges')
    for i in range(3):
        ET.SubElement(nodes, 'node', id=f'n{i}', x=str(400 * i), y='0', type='priority')
    for i, speed in enumerate((13, 6)):
        ET.SubElement(edges, 'edge', id=f'e{i}', **{
            'from': f'n{i}', 'to': f'n{i + 1}', 'numLanes': '1',
            'speed': str(speed), 'priority': '1'})
    xml(directory / 'nodes.nod.xml', nodes)
    xml(directory / 'edges.edg.xml', edges)
    checks.command('netconvert', [args.netconvert, '--node-files', 'nodes.nod.xml',
                   '--edge-files', 'edges.edg.xml', '--output-file', 'network.net.xml',
                   '--no-internal-links', 'true', '--precision', '16'], directory)
    return directory / 'network.net.xml'


def route_file(path, case):
    root = ET.Element('routes')
    ET.SubElement(root, 'vType', id='krauss', carFollowModel='Krauss', emissionClass='zero',
                  sigma=str(case['sigma']), sigmaStep=str(case['sigma_step']), tau='1',
                  accel='1.8', decel='4.5', emergencyDecel='9', length='5', minGap='2.5',
                  maxSpeed='13', speedFactor='1', speedDev='0')
    ET.SubElement(root, 'route', id='r', edges='e0 e1')
    for i, depart in enumerate((0, .3, 1.1, 2.7, 5.2)):
        v = ET.SubElement(root, 'vehicle', id=f'v{i}', type='krauss', route='r',
                          depart=str(depart), departPos='0', departSpeed='0')
        if i == 0:
            ET.SubElement(v, 'stop', lane='e0_0', endPos='70', duration='3')
        if i == 2:
            ET.SubElement(v, 'stop', lane='e1_0', endPos='60', duration='2')
    xml(path, root)


def simulate(checks, args, binary, stage, case, network):
    directory = GENERATED / stage / case['name']
    directory.mkdir(parents=True, exist_ok=True)
    route_file(directory / 'routes.rou.xml', case)
    command = [binary, '--net-file', network, '--route-files', 'routes.rou.xml',
               '--seed', str(case['seed']), '--step-length', str(STEP), '--end', '180',
               '--step-method.ballistic', str(case['ballistic']).lower(),
               '--no-step-log', 'true', '--duration-log.disable', 'true', '--precision', '16',
               '--device.rtsim.probability', '0', '--fcd-output', 'fcd.xml',
               '--fcd-output.acceleration', 'true', '--tripinfo-output', 'tripinfo.xml',
               '--save-state.times', ','.join(str(x) for x in SNAPSHOTS),
               '--save-state.files', ','.join('state_' + str(x) + '.xml' for x in SNAPSHOTS),
               '--save-state.rng', 'true', '--save-state.precision', '16']
    result = checks.command(stage + '_' + case['name'], command, directory)
    checks.check(stage + '.' + case['name'] + '.no_collision_or_teleport',
                 not any(word in result.stderr.lower() for word in ('collision', 'teleport')), stderr=result.stderr)
    trips = ET.parse(directory / 'tripinfo.xml').getroot().findall('tripinfo')
    checks.check(stage + '.' + case['name'] + '.all_arrived',
                 {x.get('id') for x in trips} == {f'v{i}' for i in range(5)})
    checks.check(stage + '.' + case['name'] + '.staggered_departures',
                 len({x.get('depart') for x in trips}) >= 3)
    steps = ET.parse(directory / 'fcd.xml').getroot().findall('timestep')
    samples = [(float(t.get('time')), v) for t in steps for v in t.findall('vehicle')]
    checks.check(stage + '.' + case['name'] + '.low_speed_start',
                 any(0 < float(v.get('speed')) < 1.8 for _, v in samples))
    checks.check(stage + '.' + case['name'] + '.stop_exercised',
                 any(t > 5 and v.get('id') == 'v0' and float(v.get('speed')) == 0 for t, v in samples))
    for time in SNAPSHOTS:
        root = ET.parse(directory / ('state_' + str(time) + '.xml')).getroot()
        checks.check(stage + '.' + case['name'] + '.saved_rng_' + str(time),
                     any('rngstate' in e.tag.lower() for e in root.iter()))
        if case['sigma_step'] > STEP:
            checks.check(stage + '.' + case['name'] + '.saved_krauss_variables_' + str(time),
                         any(e.tag == 'carFollowModel' or (e.get('id') == 'Krauss' and e.get('state') is not None)
                             for e in root.iter()))
    return directory


def compare(checks, case, candidate):
    reference = GENERATED / 'reference' / case['name']
    names = ['fcd.xml', 'tripinfo.xml'] + ['state_' + str(t) + '.xml' for t in SNAPSHOTS]
    for filename in names:
        a, b = ET.parse(reference / filename).getroot(), ET.parse(candidate / filename).getroot()
        checks.check(case['name'] + '.exact_' + filename, canonical(a) == canonical(b))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('reference', 'compare', 'all'), required=True)
    parser.add_argument('--reference-sumo', type=Path, required=True)
    parser.add_argument('--candidate-sumo', type=Path, default=ROOT.parent / 'sb/src/sumo.exe')
    parser.add_argument('--netconvert', type=Path, required=True)
    args = parser.parse_args()
    for key in ('reference_sumo', 'candidate_sumo', 'netconvert'):
        setattr(args, key, getattr(args, key).resolve())
    GENERATED.mkdir(parents=True, exist_ok=True)
    os.environ['SUMO_HOME'] = str(ROOT)
    dll = ROOT.parent / 'toolchain/xerces-install/bin'
    os.environ['PATH'] = str(dll) + os.pathsep + os.environ.get('PATH', '')
    checks = Checks()
    reference_hash = digest(args.reference_sumo)
    network = GENERATED / 'fixtures/network.net.xml'
    manifest = GENERATED / 'reference_manifest.json'
    if args.mode in ('reference', 'all'):
        network = fixtures(checks, args)
        for case in CASES:
            checks.group('reference_' + case['name'],
                         lambda case=case: simulate(checks, args, args.reference_sumo, 'reference', case, network))
        metadata = dict(reference_sha256=reference_hash, cases=CASES, network_sha256=digest(network),
                        passed=all(item['passed'] for item in checks.results))
        manifest.write_text(json.dumps(metadata, indent=2), encoding='utf-8')
    if args.mode in ('compare', 'all'):
        metadata = json.loads(manifest.read_text(encoding='utf-8'))
        checks.check('reference_completed', metadata['passed'])
        checks.check('reference_binary_unchanged', reference_hash == metadata['reference_sha256'])
        checks.check('reference_fixture_unchanged', digest(network) == metadata['network_sha256'])
        checks.check('case_matrix_unchanged', CASES == metadata['cases'])
        for case in CASES:
            def exercise(case=case):
                candidate = simulate(checks, args, args.candidate_sumo, 'candidate', case, network)
                compare(checks, case, candidate)
            checks.group('candidate_' + case['name'], exercise)
    report = dict(created=datetime.datetime.now(datetime.timezone.utc).isoformat(), mode=args.mode,
                  reference_binary=str(args.reference_sumo), reference_sha256=reference_hash,
                  candidate_binary=str(args.candidate_sumo),
                  candidate_sha256=digest(args.candidate_sumo) if args.mode != 'reference' else None,
                  purpose='Exact Krauss behavior/RNG preservation across shared dawdling-helper extraction; no RTSIm device enabled.',
                  cases=CASES, checks=checks.results, commands=checks.commands,
                  passed=bool(checks.results) and all(x['passed'] for x in checks.results))
    filename = GENERATED / ('report_' + args.mode + '.json')
    filename.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(f"{sum(x['passed'] for x in checks.results)}/{len(checks.results)} checks passed; {filename}", flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
