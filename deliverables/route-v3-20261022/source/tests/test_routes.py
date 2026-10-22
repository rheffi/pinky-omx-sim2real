import json
import math
from pathlib import Path
import unittest

from pinky_motion.geometry import dock_pose, valid_pose, overlaps, footprint
from pinky_motion.route_paths import (STATIONS, join, transfer, angle_delta, station_paths,
                                      home_paths, home_to_station, station_to_home, CY)
from pinky_motion.route_validation import check_path, pair_step_clear
from pinky_motion.routes import graph
from pinky_motion.fleet_reference import run_exchange, Plan, TimedPath, plans_clear

CONFIG=json.loads((Path(__file__).resolve().parents[1]/'config/layout.json').read_text(encoding='utf-8'))


class RouteGeometryTests(unittest.TestCase):
    def test_all_station_pairs_connect_with_basket_and_shelves(self):
        for i,a in enumerate(STATIONS):
            for j,b in enumerate(STATIONS):
                if a==b: continue
                with self.subTest(source=a,target=b):
                    segments=transfer(CONFIG,a,b); poses=join(segments)
                    self.assertLess(math.dist(poses[0][:2],dock_pose(CONFIG,i)[:2]),1e-9)
                    self.assertLess(math.dist(poses[-1][:2],dock_pose(CONFIG,j)[:2]),1e-9)
                    self.assertLess(abs(angle_delta(poses[-1][2],-math.pi/2)),1e-9)
                    self.assertTrue(check_path(CONFIG,poses,.02)['valid'])
                    self.assertEqual([s.name for s in segments if s.direction==-1],['dock_reverse'])

    def test_rotation_collision_between_clear_endpoints_is_rejected(self):
        a=(.6,-.9,0); b=(.6,-.9,-math.pi)
        self.assertTrue(valid_pose(CONFIG,a))
        self.assertTrue(valid_pose(CONFIG,b))
        self.assertFalse(check_path(CONFIG,[a,b],0)['valid'])

    def test_robots_cannot_cross_between_clear_endpoints(self):
        a=(.5,-1.2,0); b=(1.2,-1.2,math.pi)
        self.assertFalse(overlaps(footprint(CONFIG,a),footprint(CONFIG,b)))
        self.assertFalse(pair_step_clear(CONFIG,a,b,b,a,0))

    def test_graph_has_connected_incoming_and_outgoing_paths(self):
        g=graph(CONFIG)
        for edge in g['edges']:
            start=g['nodes'][edge['from']]['pose']; end=g['nodes'][edge['to']]['pose']
            poses=edge['reference_poses']
            self.assertLess(math.dist(start[:2],poses[0][:2]),1e-8)
            self.assertLess(math.dist(end[:2],poses[-1][:2]),1e-8)
            self.assertLess(abs(angle_delta(start[2],poses[0][2])),1e-8)
            self.assertLess(abs(angle_delta(end[2],poses[-1][2])),1e-8)
        for station in STATIONS:
            self.assertTrue(any(e['to']==station+'_dock' for e in g['edges']))
            self.assertTrue(any(e['from']==station+'_dock' for e in g['edges']))

    def test_station_exits_do_not_cross_lower_lane(self):
        for station in STATIONS:
            poses=join(station_paths(CONFIG,station)['exit'])
            self.assertGreaterEqual(min(p[1] for p in poses),CY-1e-9)

    def test_home_routes_and_other_parked_robot_clearance(self):
        home=home_paths()
        for station in STATIONS:
            self.assertTrue(check_path(CONFIG,join(home_to_station(CONFIG,station)))['valid'])
            for slot in ('back','front'):
                path=join(station_to_home(CONFIG,station,slot))
                self.assertTrue(check_path(CONFIG,path)['valid'])
                self.assertLess(math.dist(path[-1][:2],home[slot][:2]),1e-8)
        for key,other in [('arrival','front'),('departure','back')]:
            path=join(home[key]); fixed=home[other]
            self.assertTrue(all(pair_step_clear(CONFIG,a,b,fixed,fixed)
                                for a,b in zip(path,path[1:])))

    def test_queue_cannot_advance_through_occupied_head(self):
        home=home_paths()
        advance=Plan(home['back'],TimedPath([home['advance']]))
        self.assertFalse(plans_clear(CONFIG,advance,Plan(home['front']),0))


class TwoRobotTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.exchange=run_exchange(CONFIG)
        cls.fault=run_exchange(CONFIG,fault='stale_exit')

    def test_second_robot_waits_for_clearance_and_merge_window(self):
        events=self.exchange['events']
        first_exit=next(e for e in events if e['event']=='exit_observation' and e['robot']=='pinky1')
        second=next(e for e in events if e['event']=='departure_scheduled' and e['robot']=='pinky2')
        self.assertTrue(first_exit['released'])
        self.assertGreater(second['start'],first_exit['t'])
        self.assertGreater(self.exchange['simultaneous_movement_seconds'],0)
        self.assertFalse(self.exchange['owners'])
        self.assertTrue(all(v=='complete' for v in self.exchange['frames'][-1]['states'].values()))

    def test_fault_does_not_admit_second_or_erase_occupancy(self):
        self.assertEqual(self.fault['status'],'held_on_stale_exit')
        self.assertEqual(set(self.fault['owners'].values()),{'pinky1'})
        self.assertEqual(self.fault['queue'],['pinky2'])
        self.assertFalse(any(e['event']=='departure_scheduled' and e['robot']=='pinky2' for e in self.fault['events']))
        stopped=[f for f in self.fault['frames'] if set(f['states'].values())=={'fault_stop'}]
        self.assertGreater(len(stopped),1)
        self.assertTrue(all(f['poses']==stopped[0]['poses'] for f in stopped))

    def test_home_loading_handoff_and_return_are_all_replayed(self):
        frames=self.exchange['frames']; events=self.exchange['events']; home=home_paths()
        for rid,slot in [('pinky1','front'),('pinky2','back')]:
            self.assertLess(math.dist(frames[0]['poses'][rid][:2],home[slot][:2]),1e-8)
            self.assertLess(math.dist(frames[-1]['poses'][rid][:2],home[slot][:2]),1e-8)
            loading=next(e for e in events if e['robot']==rid and e['event']=='loading_done')
            request=next(e for e in events if e['robot']==rid and e['event']=='request_handoff')
            self.assertLess(loading['t'],request['t'])
        departures=[e for e in events if e['event']=='home_departure_scheduled']
        self.assertEqual([e['robot'] for e in departures],['pinky1','pinky2'])
        self.assertLess(departures[0]['start'],departures[1]['start'])
        self.assertTrue(any(f['states']['pinky2']=='queue_advancing' for f in frames))


if __name__=='__main__': unittest.main()
