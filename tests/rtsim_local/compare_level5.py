#!/usr/bin/env python3
"""Matched Level-5 Krauss versus native Plexe Ploeg benchmark, local only.

Both runs use the same CC leader and deterministic benchmark-only acceleration
requests. Followers use Krauss(sigma=0,tau=.6) or Plexe Ploeg(h=.6) with the shared
Krauss sigma operation explicitly set to zero. This measures model differences;
it does not validate either model, force their trajectories to agree, or estimate
automation fuel savings. Original emission datasets are read, never modified.
"""
# Copyright (C) 2026 RTSIm contributors.
# SPDX-License-Identifier: EPL-2.0 OR GPL-2.0-or-later
import argparse
import csv
import datetime
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import subprocess
import sys
import time
import traceback
import xml.etree.ElementTree as ET

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
IDS = ('leader', 'follower1', 'follower2')
FOLLOWERS = IDS[1:]
DT = 0.1
HORIZON = 180.0
LENGTH = 6.534
INITIAL_GAP = 25.0
INITIAL_SPEED = 10.0
DESIRED_SPEED = 25.0
TAU = 0.6
EMISSION_CLASS = 'PHEMlight/HDV_RT_II_D_EU0'
FIELDS = ('scenario', 'model', 'time_s', 'elapsed_s', 'vehicle', 'speed_mps',
          'acceleration_mps2', 'x_m', 'distance_m', 'fuel_mg_s', 'net_gap_m',
          'leader_request_mps2')


def dump(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def xml(path, root):
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def make_network(args):
    directory = args.output / 'fixtures'
    directory.mkdir(parents=True, exist_ok=True)
    nodes, edges = ET.Element('nodes'), ET.Element('edges')
    ET.SubElement(nodes, 'node', id='start', x='0', y='0', z='0', type='priority')
    ET.SubElement(nodes, 'node', id='end', x='8000', y='0', z='0', type='priority')
    ET.SubElement(edges, 'edge', id='road', **{'from': 'start', 'to': 'end',
                  'numLanes': '1', 'speed': '30', 'priority': '1'})
    xml(directory / 'nodes.nod.xml', nodes)
    xml(directory / 'edges.edg.xml', edges)
    network = directory / 'network.net.xml'
    command = [str(args.netconvert), '--node-files', str(directory / 'nodes.nod.xml'),
               '--edge-files', str(directory / 'edges.edg.xml'), '--output-file', str(network),
               '--no-internal-links', 'true', '--precision', '12']
    result = subprocess.run(command, capture_output=True, text=True, errors='replace', timeout=60)
    (directory / 'netconvert.stdout.txt').write_text(result.stdout, encoding='utf-8')
    (directory / 'netconvert.stderr.txt').write_text(result.stderr, encoding='utf-8')
    if result.returncode:
        raise RuntimeError('netconvert failed: ' + result.stderr)
    return network


def make_routes(path, model):
    root = ET.Element('routes')
    common = dict(vClass='truck', emissionClass=EMISSION_CLASS, length=str(LENGTH),
                  width='2.5', accel='1.5', decel='4.5', emergencyDecel='9',
                  minGap='2.5', maxSpeed=str(DESIRED_SPEED), speedFactor='1',
                  speedDev='0', sigma='0', tau=str(TAU), actionStepLength=str(DT))
    # Original legacy class supplies identical Cd/Fr0 and physical emission data.
    # No position-dependent drag or road differences are injected into this comparison.
    ET.SubElement(root, 'vType', id='leader_type', carFollowModel='CC', lanesCount='1',
                  tauEngine='0.5', **common)
    if model == 'krauss':
        ET.SubElement(root, 'vType', id='follower_type', carFollowModel='Krauss', **common)
    else:
        ET.SubElement(root, 'vType', id='follower_type', carFollowModel='CC', lanesCount='1',
                      tauEngine='0.5', ploegH=str(TAU), ploegKp='0.2', ploegKd='0.7', **common)
    ET.SubElement(root, 'route', id='route', edges='road')
    for index, identifier in enumerate(IDS):
        ET.SubElement(root, 'vehicle', id=identifier,
                      type='leader_type' if index == 0 else 'follower_type', route='route',
                      depart='0', departLane='0', departPos=format(200-index*(LENGTH+INITIAL_GAP), '.12g'),
                      departSpeed=str(INITIAL_SPEED), insertionChecks='none')
    xml(path, root)


def leader_request(scenario, elapsed):
    # Native CC fixedAcceleration is a benchmark drive-cycle input, identical in
    # both models. Its original first-order engine lag remains in the simulation.
    if 5 <= elapsed < 20:
        return 1.0
    if scenario == 'brake_recovery':
        if 70 <= elapsed < 80:
            return -1.0
        if 100 <= elapsed < 110:
            return 1.0
    return 0.0


def parse_tripinfo(path):
    trips = {}
    if not path.exists():
        return trips
    for item in ET.parse(path).getroot().findall('tripinfo'):
        emissions = item.find('emissions')
        grams = float(emissions.get('fuel_abs')) / 1000 if emissions is not None else None
        distance = float(item.get('routeLength'))
        trips[item.get('id')] = dict(route_length_m=distance, duration_s=float(item.get('duration')),
                                    arrival_s=float(item.get('arrival')), fuel_g=grams,
                                    fuel_g_per_km=grams/(distance/1000) if grams is not None and distance > 0 else None)
    return trips


def run_case(args, network, scenario, model):
    import traci
    import traci.constants as tc
    name = scenario + '_' + model
    directory = args.output / name
    directory.mkdir(parents=True, exist_ok=True)
    make_routes(directory / 'routes.rou.xml', model)
    command = [str(args.sumo), '--net-file', str(network), '--route-files', str(directory/'routes.rou.xml'),
               '--phemlight-path', str(args.phemlight_path), '--step-length', str(DT), '--threads', '1',
               '--seed', str(args.seed), '--end', '500', '--no-step-log', 'true', '--duration-log.disable', 'true',
               '--precision', '12', '--emissions.volumetric-fuel', 'false', '--device.emissions.probability', '1',
               '--tripinfo-output', str(directory/'tripinfo.xml'), '--tripinfo-output.write-unfinished', 'true',
               '--collision-output', str(directory/'collisions.xml'), '--collision.action', 'warn',
               '--time-to-teleport', '-1', '--error-log', str(directory/'sumo.errors.txt')]
    rows, commands, collisions, teleports, arrivals = [], [], [], [], {}
    started = time.perf_counter()
    connection = None
    outcome = {'name': name, 'directory': str(directory), 'argv': command}
    last_request = None
    try:
        with (directory/'sumo.stdout.txt').open('w', encoding='utf-8') as stdout:
            traci.start(command, label=name, stdout=stdout, numRetries=5)
            connection = traci.getConnection(name)
            connection.simulationStep()
            initial_time = connection.simulation.getTime()
            if set(connection.vehicle.getIDList()) != set(IDS):
                raise RuntimeError('All three vehicles must be inserted before the drive cycle')

            def set_parameter(identifier, key, value):
                connection.vehicle.setParameter(identifier, 'carFollowModel.'+key, str(value))
                commands.append(dict(time_s=connection.simulation.getTime(), vehicle=identifier, key=key, value=str(value)))

            for identifier in IDS:
                connection.vehicle.setLaneChangeMode(identifier, 0)
            set_parameter('leader', 'ccac', 1)
            set_parameter('leader', 'ccds', DESIRED_SPEED)
            set_parameter('leader', 'ccfa', '1:0')
            parameters = {}
            for index, identifier in enumerate(FOLLOWERS, start=1):
                if model == 'ploeg':
                    set_parameter(identifier, 'ccds', DESIRED_SPEED)
                    set_parameter(identifier, 'ccph', TAU)
                    set_parameter(identifier, 'ccpkp', 0.2)
                    set_parameter(identifier, 'ccpkd', 0.7)
                    set_parameter(identifier, 'ccca', 1)
                    set_parameter(identifier, 'ccaf', '1:leader:'+IDS[index-1])
                    set_parameter(identifier, 'ccac', 4)
                    set_parameter(identifier, 'rtsim.sigmaStep', DT)
                    set_parameter(identifier, 'rtsim.sigma', 0)
                    active = connection.vehicle.getParameter(identifier, 'carFollowModel.ccac')
                    sigma = connection.vehicle.getParameter(identifier, 'carFollowModel.rtsim.sigma')
                    if int(active) != 4 or float(sigma) != 0:
                        raise RuntimeError('Requested Ploeg/shared-sigma settings were not active')
                    parameters[identifier] = dict(controller='Plexe Ploeg', controller_id=int(active),
                        sigma=float(sigma), sigma_step_s=DT, h_s=TAU, h_verification='XML ploegH plus successful ccph setter; upstream has no ccph getter',
                        kp=0.2, kd=0.7, auto_feed_leader='leader', auto_feed_front=IDS[index-1],
                        use_controller_acceleration=True, engine='native first-order lag', tau_engine_s=0.5)
                else:
                    actual_tau = connection.vehicle.getTau(identifier)
                    actual_sigma = connection.vehicle.getImperfection(identifier)
                    if actual_tau != TAU or actual_sigma != 0:
                        raise RuntimeError('Krauss parameter verification failed')
                    parameters[identifier] = dict(controller='Krauss', tau_s=actual_tau, sigma=actual_sigma)
                parameters[identifier].update(length_m=connection.vehicle.getLength(identifier),
                    min_gap_m=connection.vehicle.getMinGap(identifier), accel_mps2=connection.vehicle.getAccel(identifier),
                    decel_mps2=connection.vehicle.getDecel(identifier))
            outcome['parameters'] = parameters
            vehicle_vars = (tc.VAR_SPEED, tc.VAR_ACCELERATION, tc.VAR_POSITION, tc.VAR_DISTANCE, tc.VAR_FUELCONSUMPTION)
            for identifier in IDS:
                connection.vehicle.subscribe(identifier, vehicle_vars)
            connection.simulation.subscribe((tc.VAR_TIME, tc.VAR_MIN_EXPECTED_VEHICLES,
                tc.VAR_ARRIVED_VEHICLES_IDS, tc.VAR_COLLIDING_VEHICLES_IDS, tc.VAR_TELEPORT_STARTING_VEHICLES_IDS))

            def record(elapsed, request):
                all_values = connection.vehicle.getAllSubscriptionResults()
                for index, identifier in enumerate(IDS):
                    if identifier not in all_values:
                        continue
                    values = all_values[identifier]
                    front = all_values.get(IDS[index-1]) if index else None
                    gap = front[tc.VAR_POSITION][0]-LENGTH-values[tc.VAR_POSITION][0] if front else None
                    row = dict(scenario=scenario, model=model, time_s=round(initial_time+elapsed, 10),
                               elapsed_s=round(elapsed, 10), vehicle=identifier, speed_mps=values[tc.VAR_SPEED],
                               acceleration_mps2=values[tc.VAR_ACCELERATION], x_m=values[tc.VAR_POSITION][0],
                               distance_m=values[tc.VAR_DISTANCE], fuel_mg_s=values[tc.VAR_FUELCONSUMPTION],
                               net_gap_m=gap, leader_request_mps2=request)
                    if any(not math.isfinite(row[key]) for key in ('speed_mps','acceleration_mps2','x_m','distance_m','fuel_mg_s')):
                        raise RuntimeError('Nonfinite trajectory or emission value')
                    rows.append(row)

            record(0.0, 0.0)
            for step in range(1, 5000):
                elapsed_before = round((step-1)*DT, 10)
                request = leader_request(scenario, elapsed_before)
                if request != last_request and 'leader' not in arrivals:
                    set_parameter('leader', 'ccfa', '1:'+str(request))
                    last_request = request
                connection.simulationStep()
                state = connection.simulation.getSubscriptionResults()
                elapsed = round(state[tc.VAR_TIME]-initial_time, 10)
                for identifier in state[tc.VAR_ARRIVED_VEHICLES_IDS]:
                    arrivals[identifier] = state[tc.VAR_TIME]
                if state[tc.VAR_COLLIDING_VEHICLES_IDS]:
                    collisions.append(dict(time_s=state[tc.VAR_TIME], ids=list(state[tc.VAR_COLLIDING_VEHICLES_IDS])))
                if state[tc.VAR_TELEPORT_STARTING_VEHICLES_IDS]:
                    teleports.append(dict(time_s=state[tc.VAR_TIME], ids=list(state[tc.VAR_TELEPORT_STARTING_VEHICLES_IDS])))
                record(elapsed, request)
                if state[tc.VAR_MIN_EXPECTED_VEHICLES] == 0:
                    break
                if elapsed >= 499:
                    break
            outcome.update(completed=set(arrivals)==set(IDS), arrivals=arrivals,
                           collision_events=collisions, teleport_events=teleports)
            connection.close()
            connection = None
    except Exception as error:
        outcome.update(completed=False, error=str(error), traceback=traceback.format_exc(),
                       arrivals=arrivals, collision_events=collisions, teleport_events=teleports)
    finally:
        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass
    outcome['wall_seconds'] = time.perf_counter()-started
    outcome['samples'] = len(rows)
    outcome['tripinfo'] = parse_tripinfo(directory/'tripinfo.xml')
    outcome['horizon_complete'] = all(any(r['vehicle']==identifier and abs(r['elapsed_s']-HORIZON)<1e-8 for r in rows) for identifier in IDS)
    errors_path = directory/'sumo.errors.txt'
    outcome['diagnostics'] = errors_path.read_text(encoding='utf-8', errors='replace') if errors_path.exists() else ''
    outcome['valid'] = (outcome['completed'] and outcome['horizon_complete'] and not collisions and not teleports and 'error' not in outcome)
    with (directory/'trajectories.csv').open('w', encoding='utf-8', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    dump(directory/'commands.json', commands)
    dump(directory/'run.json', outcome)
    print(name+': completed='+str(outcome['completed'])+', valid='+str(outcome['valid'])+', wall_s='+format(outcome['wall_seconds'],'.2f'), flush=True)
    return outcome, rows


def mean(values):
    return statistics.fmean(values) if values else None


def errors(values):
    return dict(rmse=math.sqrt(mean([v*v for v in values])), mae=mean([abs(v) for v in values]),
                max_abs=max(abs(v) for v in values), mean_signed=mean(values)) if values else None


def percent(left, right):
    return 100*(right-left)/left if left not in (None,0) and right is not None else None


def summarize_vehicle(rows, identifier):
    data = [r for r in rows if r['vehicle']==identifier and r['elapsed_s'] <= HORIZON]
    gaps = [r['net_gap_m'] for r in data if r['net_gap_m'] is not None]
    grams = sum(r['fuel_mg_s']*DT/1000 for r in data if r['elapsed_s']>0)
    distance = data[-1]['distance_m']-data[0]['distance_m'] if data else 0
    steady = {}
    for name, lo, hi in (('initial_cruise',55,65), ('final_cruise',150,170)):
        window = [r for r in data if lo<=r['elapsed_s']<=hi]
        steady[name] = dict(start_s=lo, end_s=hi, speed_mean_mps=mean([r['speed_mps'] for r in window]),
                           gap_mean_m=mean([r['net_gap_m'] for r in window if r['net_gap_m'] is not None]))
    return dict(samples=len(data), minimum_speed_mps=min((r['speed_mps'] for r in data),default=None),
                net_gap_min_m=min(gaps,default=None), net_gap_mean_m=mean(gaps), steady_windows=steady,
                horizon_fuel_g=grams, horizon_distance_m=distance,
                horizon_fuel_g_per_km=grams/(distance/1000) if distance>0 else None)


def compare(args, scenario, left, right):
    left_run, left_rows = left
    right_run, right_rows = right
    a = {(r['elapsed_s'],r['vehicle']):r for r in left_rows if r['elapsed_s']<=HORIZON}
    b = {(r['elapsed_s'],r['vehicle']):r for r in right_rows if r['elapsed_s']<=HORIZON}
    leader_a = {key:row for key,row in a.items() if key[1]=='leader'}
    leader_b = {key:row for key,row in b.items() if key[1]=='leader'}
    keys = sorted(set(leader_a)&set(leader_b))
    leader_diffs = {field:max((abs(a[key][field]-b[key][field]) for key in keys),default=None)
                    for field in ('speed_mps','acceleration_mps2','x_m','distance_m','fuel_mg_s')}
    leader_identical = bool(keys) and leader_a.keys()==leader_b.keys() and all(x==0 for x in leader_diffs.values())
    result = dict(scenario=scenario, horizon_s=HORIZON, leader_identical=leader_identical,
                  leader_max_absolute_differences=leader_diffs,
                  valid_matched_comparison=leader_identical and left_run['valid'] and right_run['valid'], followers={})
    pairs = []
    for identifier in FOLLOWERS:
        common_keys = sorted(key for key in set(a)&set(b) if key[1]==identifier)
        speed = errors([b[key]['speed_mps']-a[key]['speed_mps'] for key in common_keys])
        acceleration = errors([b[key]['acceleration_mps2']-a[key]['acceleration_mps2'] for key in common_keys])
        gap = errors([b[key]['net_gap_m']-a[key]['net_gap_m'] for key in common_keys
                      if a[key]['net_gap_m'] is not None and b[key]['net_gap_m'] is not None])
        summaries = {model:summarize_vehicle(rows,identifier) for model,rows in (('krauss',left_rows),('ploeg',right_rows))}
        trips = {model:run['tripinfo'].get(identifier) for model,run in (('krauss',left_run),('ploeg',right_run))}
        horizon_delta = percent(summaries['krauss']['horizon_fuel_g_per_km'], summaries['ploeg']['horizon_fuel_g_per_km'])
        total_delta = percent(trips['krauss']['fuel_g'],trips['ploeg']['fuel_g']) if all(trips.values()) else None
        normalized_delta = percent(trips['krauss']['fuel_g_per_km'],trips['ploeg']['fuel_g_per_km']) if all(trips.values()) else None
        result['followers'][identifier] = dict(common_samples=len(common_keys), speed_mps=speed,
            acceleration_mps2=acceleration, net_gap_m=gap, runs=summaries, full_trip=trips,
            horizon_fuel_g_per_km_percent_difference=horizon_delta,
            full_trip_fuel_g_percent_difference=total_delta,
            full_trip_fuel_g_per_km_percent_difference=normalized_delta)
        for key in common_keys:
            pair = dict(elapsed_s=key[0],vehicle=identifier)
            for field in ('speed_mps','acceleration_mps2','net_gap_m','fuel_mg_s','distance_m'):
                pair['krauss_'+field]=a[key][field]
                pair['ploeg_'+field]=b[key][field]
            pairs.append(pair)
    if pairs:
        with (args.output/(scenario+'_paired.csv')).open('w',encoding='utf-8',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(pairs[0]))
            writer.writeheader()
            writer.writerows(pairs)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sumo',type=Path,default=ROOT.parent/'sigma-runtime/sumo.exe')
    parser.add_argument('--netconvert',type=Path,default=ROOT.parent/'sigma-runtime/netconvert.exe')
    parser.add_argument('--phemlight-path',type=Path,default=Path(__file__).resolve().parent/'generated_legacy/datasets/original')
    parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parent/'generated_level5')
    parser.add_argument('--seed',type=int,default=42)
    args=parser.parse_args()
    for name in ('sumo','netconvert','phemlight_path','output'):
        setattr(args,name,getattr(args,name).resolve())
    args.output.mkdir(parents=True,exist_ok=True)
    os.environ['SUMO_HOME']=str(ROOT)
    os.environ['PATH']=str(ROOT.parent/'toolchain/xerces-install/bin')+os.pathsep+os.environ.get('PATH','')
    sys.path.insert(0,str(ROOT/'tools'))
    dataset_files=[args.phemlight_path/('HDV_RT_II_D_EU0'+suffix) for suffix in ('.PHEMLight.veh','.csv','_FC.csv')]
    hashes={str(path):digest(path) for path in dataset_files}
    report=dict(created_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        scope='Deterministic matched-controller comparison; synthetic flat straight road and copied original RTSim truck emission data. No empirical validation or fuel-savings claim.',
        sumo=str(args.sumo),sumo_sha256=digest(args.sumo),dataset_hashes=hashes,step_length_s=DT,seed=args.seed,
        vehicle_length_m=LENGTH,initial_net_gap_m=INITIAL_GAP,initial_speed_mps=INITIAL_SPEED,
        desired_speed_mps=DESIRED_SPEED,tau_or_h_s=TAU,sigma=0,emission_class=EMISSION_CLASS,
        fixed_identical_road_loads=dict(Cd=0.636723,Fr0=0.006923,frontal_area_m2=5.92,mass_kg=9507,loading_kg=10000),
        model_distinctions=['Krauss tau is its native safe-speed headway; Ploeg h belongs to its native differential controller.',
            'Ploeg uses the unchanged target 2+h*v in its gap input; SUMO minGap is kept at 2.5m for both models, not tuned to erase the extra spacing offset.',
            'Ploeg keeps its original 0.5s first-order engine and gains kp=.2 kd=.7; Krauss keeps its own speed update.',
            'The identical CC leader receives piecewise fixedAcceleration requests solely to define this benchmark; follower speeds are not forced.',
            'Fuel is reported as mass, total grams and grams/km; a model difference is not an automation fuel saving.'],
        drive_cycle='0-5s command0; 5-20s +1m/s2; otherwise0. Braking case adds70-80s -1m/s2 and100-110s +1m/s2; engine remains active.',
        horizon_s=HORIZON,runs={},comparisons=[])
    started=time.perf_counter()
    try:
        network=make_network(args)
        report['network_sha256']=digest(network)
        for scenario in ('acceleration_cruise','brake_recovery'):
            cases={}
            for model in ('krauss','ploeg'):
                cases[model]=run_case(args,network,scenario,model)
                report['runs'][scenario+'_'+model]=cases[model][0]
            report['comparisons'].append(compare(args,scenario,cases['krauss'],cases['ploeg']))
            dump(args.output/'comparison.json',report)
    except Exception as error:
        report.update(error=str(error),traceback=traceback.format_exc())
    report['wall_seconds']=time.perf_counter()-started
    report['dataset_unchanged']=hashes=={str(path):digest(path) for path in dataset_files}
    report['binary_unchanged']=report['sumo_sha256']==digest(args.sumo)
    report['valid']=len(report['comparisons'])==2 and all(item['valid_matched_comparison'] for item in report['comparisons']) and report['dataset_unchanged'] and report['binary_unchanged']
    dump(args.output/'comparison.json',report)
    print('Matched benchmark valid='+str(report['valid'])+'; '+str(args.output/'comparison.json'),flush=True)
    return 0 if report['valid'] else 1


if __name__=='__main__':
    if hasattr(sys.stdout,'reconfigure'):
        sys.stdout.reconfigure(encoding='utf-8')
    sys.exit(main())
