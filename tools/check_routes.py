"""Persist geometry, waits and two-robot kinematic reference evidence."""
import argparse
from itertools import permutations
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from pinky_motion.route_paths import (STATIONS,join,transfer,station_paths,home_paths,
                                      home_to_station,station_to_home,ROUTE_VERSION,
                                      loop_leg,LOOP_LENGTH)
from pinky_motion.route_validation import check_path,pair_step_clear
from pinky_motion.fleet_reference import run_exchange
from pinky_motion.geometry import footprint,overlaps
from pinky_motion.routes import graph


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args(); args.output.mkdir(parents=True,exist_ok=False)
    config=json.loads((ROOT/'config/layout.json').read_text(encoding='utf-8'))
    result={'kind':'geometric_and_kinematic_reference_validation','physical_simulation':False,
            'hardware_connected':False,'route_version':ROUTE_VERSION,
            'config':config,'routes':{},'waits':{},'home_routes':{}}
    path_data={}
    for a,b in permutations(STATIONS,2):
        segments=transfer(config,a,b); poses=join(segments); key=a+'_to_'+b
        result['routes'][key]=check_path(config,poses,.02)
        path_data[key]=[{'name':s.name,'direction':s.direction,'poses':s.poses} for s in segments]
    home=home_paths()
    home_segments={'queue_advance':[home['advance']]}
    for station in STATIONS:
        home_segments['home_to_'+station]=home_to_station(config,station)
        for slot in ('back','front'):
            home_segments[station+'_to_home_'+slot]=station_to_home(config,station,slot)
    for key,segments in home_segments.items():
        result['home_routes'][key]=check_path(config,join(segments),.02)
        path_data[key]=[{'name':s.name,'direction':s.direction,'poses':s.poses} for s in segments]
    # Split the full loop because loop_leg(s,s) intentionally has zero length.
    loop=join([loop_leg(0,LOOP_LENGTH/2),loop_leg(LOOP_LENGTH/2,0)])
    result['home_waits']={}
    for slot in ('back','front'):
        fixed=home[slot]
        result['home_waits'][slot]={'pose':fixed,'loop_clear':all(
            pair_step_clear(config,a,b,fixed,fixed,.02) for a,b in zip(loop,loop[1:]))}
    result['home_queue_checks']={
        'return_while_front_occupied':all(pair_step_clear(config,a,b,home['front'],home['front'])
            for a,b in zip(join(home['arrival']),join(home['arrival'])[1:])),
        'depart_while_back_occupied':all(pair_step_clear(config,a,b,home['back'],home['back'])
            for a,b in zip(join(home['departure']),join(home['departure'])[1:]))}
    # Full mission paths must avoid the OTHER loading bay's waiting footprint.
    for moving,waiting in (('A','B'),('B','A')):
        fixed=station_paths(config,waiting)['wait_pose']
        checks=[]
        for a,b in ((moving,'handoff'),('handoff',moving)):
            poses=join(transfer(config,a,b))
            checks.extend(pair_step_clear(config,p,q,fixed,fixed,.02) for p,q in zip(poses,poses[1:]))
        result['waits'][waiting]={'pose':fixed,'other_robot_roundtrip_clear':all(checks)}
    # A traffic lane is not a bay; record this tempting but obstructive wait.
    obstruction=(.9,-.95,3.141592653589793)
    handoff_to_A=join(transfer(config,'handoff','A'))
    result['reject_wait_in_upper_lane']=any(overlaps(footprint(config,p,.02),footprint(config,obstruction,.02)) for p in handoff_to_A)
    for name,fault in [('exchange',None),('stale_exit','stale_exit')]:
        run=run_exchange(config,fault=fault)
        frames=run.pop('frames')
        (args.output/(name+'-frames.json')).write_text(json.dumps(frames,separators=(',',':')),encoding='utf-8')
        result[name]=run
    result['limitations']=['Reference-path interpolation and synthetic observations, not Isaac wheel dynamics.',
                          '20mm geometric expansion per robot is an assumption, not measured tracking/braking accuracy.',
                          'Lower-left home queue is FIFO, not independently accessible robot-assigned bays.',
                          'One loading station per robot in this example; loaded robots wait in A/B.',
                          'Loading and unloading are 3-second events; no camera, ACT or arm execution.',
                          'Nominal timed plans require new observation handling before hardware use.']
    result['passed']=(all(r['valid'] for r in result['routes'].values())
                      and all(r['other_robot_roundtrip_clear'] for r in result['waits'].values())
                      and all(result['exchange']['checks'].values()) and all(result['stale_exit']['checks'].values())
                      and all(r['valid'] for r in result['home_routes'].values())
                      and all(r['loop_clear'] for r in result['home_waits'].values())
                      and all(result['home_queue_checks'].values())
                      and result['reject_wait_in_upper_lane'])
    (args.output/'routes.json').write_text(json.dumps(path_data,separators=(',',':')),encoding='utf-8')
    (args.output/'route-graph.json').write_text(json.dumps(graph(config),separators=(',',':')),encoding='utf-8')
    (args.output/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'passed':result['passed'],'routes':len(result['routes']),
                      'elapsed_sim_seconds':result['exchange']['elapsed_sim_seconds'],
                      'simultaneous_movement_seconds':result['exchange']['simultaneous_movement_seconds'],
                      'events':result['exchange']['events']},ensure_ascii=False))
    raise SystemExit(0 if result['passed'] else 1)


if __name__=='__main__': main()
