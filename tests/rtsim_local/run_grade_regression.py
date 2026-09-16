#!/usr/bin/env python3
"""Check the PHEMlight adapters against gravity along an inclined road.

Build SUMO's emissionsDrivingCycle target, then run this script with
--emissions-driving-cycle /path/to/emissionsDrivingCycle. The test copies the
bundled sample files into --output-dir and replaces their road-load and fuel
maps with deliberately simple, synthetic fixtures. They are not research data.
A linear fuel map makes reported fuel invertible to engine/wheel power; this
checks the actual compiled adapters without duplicating their implementation.
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

ROOT = Path(__file__).resolve().parents[2]
MASS = 2000.0
LOAD = 500.0
TOTAL_MASS = MASS + LOAD
RATED_POWER = 100.0
GRAVITY = 9.81
ROLLING = 0.01
EFFICIENCY = 0.9
SPEED = 20.0
FC_INTERCEPT = 1000.0
FC_SLOPE = 1000.0
ANGLES = (('flat', 0.0), ('negative_zero', -0.0),
          ('up_3_degrees', 3.0), ('down_3_degrees', -3.0),
          ('up_3_percent', math.degrees(math.atan(0.03))),
          ('down_3_percent', -math.degrees(math.atan(0.03))))
FIELDS = ('time', 'speed', 'acceleration', 'slope', 'CO', 'CO2',
          'HC', 'PMx', 'NOx', 'fuel', 'electricity', 'coasting')


class Checks:
    def __init__(self):
        self.results = []

    def check(self, name, passed, **detail):
        self.results.append(dict(name=name, passed=bool(passed), **detail))

    def near(self, name, actual, expected, tolerance):
        self.check(name, math.isfinite(actual) and abs(actual - expected) <= tolerance,
                   actual=actual, expected=expected, absolute_tolerance=tolerance)


def replace_value(lines, marker, value):
    index = lines.index(marker)
    lines[index + 1] = str(value)


def make_dataset(output, model):
    """No aero/rotational/engine drag, constant rolling load, linear fuel map."""
    stem = 'PC_D_EU4' if model == 'PHEMlight' else 'PC_EU4_D_MW'
    source = ROOT / 'data/emissions' / model
    destination = output / model
    destination.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(source / (stem + '.csv'), destination / (stem + '.csv'))
    vehicle_file = destination / (stem + '.PHEMLight.veh')
    if model == 'PHEMlight':
        lines = (source / (stem + '.PHEMLight.veh')).read_text(encoding='utf-8').splitlines()
        for marker, value in (
                ('c Vehicle mass [kg]', MASS), ('c Vehicle loading [kg]', LOAD),
                ('c Cd value [-]', 0), ('c Cross sectional area [m^2]', 0),
                ('c Wheels equivalent rotational inertia [kg] (= I_wheel/rdyn^2)', 0),
                ('c Auxiliaries base power demand (normalized) [-]', 0),
                ('c Engine rated power [kW]', RATED_POWER),
                ('c Fr0', ROLLING), ('c Fr1', 0), ('c Fr2', 0),
                ('c Fr3', 0), ('c Fr4', 0),
                ('c P_n_max_p0', 1), ('c P_n_max_p1', 1)):
            replace_value(lines, marker, value)
        table = lines.index('c vehicle speed [km/h] (converted to m/s after read-in), gear ratio [-], rotational mass factor [-]')
        drag = lines.index('c n_norm, pe_drag _norm')
        for i in range(table + 1, drag):
            cells = lines[i].split(',')
            cells[2] = '1'
            lines[i] = ','.join(cells)
        for i in range(drag + 1, len(lines)):
            cells = lines[i].split(',')
            cells[1] = '0'
            lines[i] = ','.join(cells)
        vehicle_file.write_text('\n'.join(lines) + '\n', encoding='utf-8')
    else:
        vehicle = json.loads((source / (stem + '.PHEMLight.veh')).read_text(encoding='utf-8-sig'))
        vehicle['VehicleData'].update(Mass=MASS, Loading=LOAD, RedMassWheel=0, Cw=0, A=0)
        vehicle['AuxiliariesData']['Pauxnorm'] = 0
        vehicle['EngineData']['ICEData']['Prated'] = RATED_POWER
        vehicle['RollingResData'].update(Fr0=ROLLING, Fr1=0, Fr2=0, Fr3=0, Fr4=0)
        vehicle['FLDData'].update(P_n_max_p0=1, P_n_max_p1=1)
        drag = vehicle['FLDData']['DragCurve']['pe_drag_norm']
        vehicle['FLDData']['DragCurve']['pe_drag_norm'] = [0] * len(drag)
        table = vehicle['TransmissionData']['Transm']['RotMassF']
        vehicle['TransmissionData']['Transm']['RotMassF'] = [1] * len(table)
        vehicle_file.write_text(json.dumps(vehicle, indent=2), encoding='utf-8')
    # Both FC readers normalize power and FC by rated power. Thus this map
    # gives FC[g/h] = FC_INTERCEPT * ratedPower + FC_SLOPE * enginePower[kW].
    fuel = ['c P_norm(rated),FC', '[-],[g/h/kWrated]',
            'Synthetic linear map for the degree/grade regression only',
            'idle,1000', '-1,0', '0,1000', '2,3000']
    (destination / (stem + '_FC.csv')).write_text('\n'.join(fuel) + '\n', encoding='utf-8')
    return destination, stem


def test_model(args, checks, model):
    directory, stem = make_dataset(args.output_dir, model)
    samples = []
    for name, angle in ANGLES:
        # For road speed v, dz/dt = v*sin(theta), so gravity power is m*g*dz/dt.
        gravity_accel = GRAVITY * math.sin(math.radians(angle))
        coast = -GRAVITY * ROLLING - gravity_accel
        for kind, acceleration in (('power', 0.5), ('cap', 1000.0),
                                   ('coast_below', coast - 0.0001),
                                   ('coast_above', coast + 0.0001)):
            samples.append(dict(name=name + '.' + kind, angle=angle,
                                acceleration=acceleration, kind=kind, coast=coast,
                                gravity_power_kw=TOTAL_MASS * SPEED * gravity_accel / 1000))
    timeline = directory / 'cycle.csv'
    timeline.write_text(''.join(f"{i};{SPEED};{s['acceleration']:.17g};{s['angle']:.17g}\n"
                               for i, s in enumerate(samples)), encoding='utf-8')
    output = directory / 'results.csv'
    argv = [str(args.emissions_driving_cycle), '--timeline-file', str(timeline),
            '--have-slope', '--compute-a.zero-correction',
            '--emission-class', model + '/' + stem,
            '--phemlight-path', str(directory), '--output-file', str(output),
            '--output.attributes', 'all', '--quiet']
    process = subprocess.run(argv, cwd=directory, text=True, capture_output=True,
                             errors='replace', timeout=120)
    (directory / 'stdout.txt').write_text(process.stdout, encoding='utf-8')
    (directory / 'stderr.txt').write_text(process.stderr, encoding='utf-8')
    checks.check(model + '.run', process.returncode == 0,
                 argv=argv, returncode=process.returncode, stderr=process.stderr)
    if process.returncode:
        return
    rows = [dict(zip(FIELDS, map(float, line.split(';'))))
            for line in output.read_text(encoding='utf-8').splitlines()]
    checks.check(model + '.sample_count', len(rows) == len(samples), actual=len(rows))
    if len(rows) != len(samples):
        return
    measured = {}
    for sample, row in zip(samples, rows):
        label = model + '.' + sample['name']
        measured[sample['name']] = row
        # emissionsDrivingCycle's CSV stream prints six significant digits.
        checks.near(label + '.coasting_mps2', row['coasting'], sample['coast'], 0.000005)
        if sample['kind'] == 'power':
            checks.near(label + '.acceleration_unclipped', row['acceleration'], 0.5, 0.000005)
            engine_power = (row['fuel'] * 3.6 - FC_INTERCEPT * RATED_POWER) / FC_SLOPE
            wheel_power = engine_power * EFFICIENCY
            expected = (TOTAL_MASS * SPEED * (0.5 + GRAVITY * ROLLING) / 1000
                        + sample['gravity_power_kw'])
            checks.near(label + '.wheel_power_kw', wheel_power, expected, 0.001)
            sample['measured_wheel_power_kw'] = wheel_power
            sample['expected_wheel_power_kw'] = expected
        elif sample['kind'] == 'cap':
            # Preserve the existing CEP maximum-acceleration convention: the
            # road-load engine power is subtracted before dividing by m*v.
            # This assertion tests the adapter's slope conversion in that path.
            load_kw = (TOTAL_MASS * SPEED * GRAVITY * ROLLING / 1000
                       + sample['gravity_power_kw']) / EFFICIENCY
            expected = (RATED_POWER - load_kw) * 1000 / (TOTAL_MASS * SPEED)
            checks.near(label + '.modified_accel_mps2', row['acceleration'], expected, 0.000005)
        elif sample['kind'] == 'coast_below':
            checks.check(label + '.fuel_cutoff', row['fuel'] == 0, actual=row['fuel'])
        else:
            checks.check(label + '.fuel_on', row['fuel'] > 0, actual=row['fuel'])
    checks.check(model + '.signed_zero_equivalent',
                 measured['flat.power'] | {'time': 0} == measured['negative_zero.power'] | {'time': 0})
    up_degrees = measured['up_3_degrees.power']['fuel']
    up_percent = measured['up_3_percent.power']['fuel']
    checks.check(model + '.degrees_not_percent', up_degrees > up_percent,
                 fuel_3_degrees=up_degrees, fuel_3_percent=up_percent)
    (directory / 'physical_expectations.json').write_text(json.dumps(samples, indent=2), encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--emissions-driving-cycle', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path,
                        default=Path(__file__).resolve().parent / 'generated_grade')
    args = parser.parse_args()
    args.emissions_driving_cycle = args.emissions_driving_cycle.resolve()
    args.output_dir = args.output_dir.resolve()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    os.environ['SUMO_HOME'] = str(ROOT)
    checks = Checks()
    for model in ('PHEMlight', 'PHEMlight5'):
        try:
            test_model(args, checks, model)
        except Exception as error:
            checks.check(model + '.exception', False, error=str(error))
            (args.output_dir / (model + '.exception.txt')).write_text(traceback.format_exc(), encoding='utf-8')
    report = dict(created=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  purpose='Compiled adapter regression against m*g*v*sin(theta), not empirical fuel calibration.',
                  binary=str(args.emissions_driving_cycle),
                  binary_sha256=hashlib.sha256(args.emissions_driving_cycle.read_bytes()).hexdigest(),
                  passed=bool(checks.results) and all(check['passed'] for check in checks.results),
                  checks=checks.results)
    path = args.output_dir / 'grade_regression_report.json'
    path.write_text(json.dumps(report, indent=2), encoding='utf-8')
    failures = [check for check in checks.results if not check['passed']]
    for check in failures:
        print('FAIL:', check)
    print(f"{len(checks.results) - len(failures)}/{len(checks.results)} checks passed; report: {path}")
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    sys.exit(main())
