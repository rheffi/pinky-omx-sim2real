import json
import math
from pathlib import Path
import unittest

from pinky_motion import compact_routes as routes
from pinky_motion.compact_reference import run_pipeline, order_visits
from pinky_motion.route_paths import join, angle_delta
from pinky_motion.route_validation import check_path, pair_step_clear
from pinky_motion.fleet_reference import Plan, TimedPath, plans_clear

CONFIG = json.loads((Path(__file__).resolve().parents[1]/'config/layout.json').read_text())


class CompactGeometryTests(unittest.TestCase):
    def test_full_paths_enclose_rigid_basket_and_obstacles(self):
        for name, segments in routes.route_set(CONFIG).items():
            with self.subTest(route=name):
                self.assertTrue(check_path(CONFIG, join(segments), .02)['valid'])
                # Signed velocity follows the body's axis; there is no sideways
                # translation hidden in a path drawn between arbitrary poses.
                for s in segments:
                    for a, b in zip(s.poses, s.poses[1:]):
                        dx, dy = b[0]-a[0], b[1]-a[1]
                        yaw = a[2] + angle_delta(a[2], b[2])/2
                        self.assertLess(abs(-dx*math.sin(yaw)+dy*math.cos(yaw)), .00002)
                        if s.direction:
                            self.assertGreaterEqual(s.direction*(dx*math.cos(yaw)+dy*math.sin(yaw)), -1e-8)
                        else:
                            self.assertLess(math.hypot(dx, dy), 1e-10)

    def test_departure_and_return_avoid_occupied_queue(self):
        home = routes.home_paths()
        for station in ('A', 'B', 'handoff'):
            p = join(routes.home_to_station(CONFIG, station))
            self.assertTrue(all(pair_step_clear(CONFIG, a, b, home['back'], home['back'])
                                for a, b in zip(p, p[1:])))
        p = join(routes.station_to_home(CONFIG, 'handoff', 'back'))
        self.assertTrue(all(pair_step_clear(CONFIG, a, b, home['front'], home['front'])
                            for a, b in zip(p, p[1:])))

    def test_occupied_front_blocks_queue_advance(self):
        home = routes.home_paths()
        move = Plan(home['back'], TimedPath([home['advance']]))
        self.assertFalse(plans_clear(CONFIG, move, Plan(home['front']), 0))

    def test_backward_visits_and_unfinished_returns_are_explicitly_unsupported(self):
        with self.assertRaises(ValueError): routes.transfer(CONFIG, 'B', 'A')
        with self.assertRaises(ValueError): routes.station_to_home(CONFIG, 'A')
        self.assertEqual(order_visits(('B', 'A')), ['A', 'B', 'handoff'])
        self.assertEqual(order_visits(('B',)), ['B', 'handoff'])


class CompactScenarioTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.normal = run_pipeline(CONFIG)
        cls.fault = run_pipeline(CONFIG, fault=True)

    def test_pipeline_visits_both_stations_and_releases_ownership(self):
        self.assertTrue(all(self.normal['checks'].values()))
        for rid in ('pinky1', 'pinky2'):
            destinations = [e['target'] for e in self.normal['events']
                            if e['robot'] == rid and e['event'] == 'observed_arrival'
                            and e['target'] in ('A', 'B', 'handoff')]
            self.assertEqual(destinations, ['A', 'B', 'handoff'])
        self.assertTrue(any('wait_handoff' in f['states'].values() for f in self.normal['frames']))

    def test_no_overlapping_movement_permissions(self):
        end = 0
        for event in self.normal['events']:
            if event['event'] == 'move_started':
                self.assertGreaterEqual(event['t']+1e-8, end)
                end = event['end']

    def test_handoff_released_after_first_robot_finishes_return(self):
        e = self.normal['events']
        returned = next(x for x in e if x['robot'] == 'pinky1' and x['event'] == 'observed_arrival' and x['target'] == 'front')
        admitted = next(x for x in e if x['robot'] == 'pinky2' and x['event'] == 'move_started' and x['target'] == 'handoff')
        self.assertGreaterEqual(admitted['t'], returned['t'])

    def test_stale_mid_transfer_keeps_owners_and_does_not_resume(self):
        self.assertTrue(all(self.fault['checks'].values()))
        self.assertEqual(self.fault['move_owner'], 'pinky1')
        stopped = [f for f in self.fault['frames'] if set(f['states'].values()) == {'fault_stop'}]
        self.assertGreater(len(stopped), 50)
        self.assertTrue(all(f['poses'] == stopped[0]['poses'] for f in stopped))

    def test_all_two_order_loading_subsets_finish_without_collision(self):
        variants = [('A',), ('B',), ('A', 'B')]
        for a in variants:
            for b in variants:
                with self.subTest(first=a, second=b):
                    result = run_pipeline(CONFIG, orders=(a, b))
                    self.assertTrue(all(result['checks'].values()))


if __name__ == '__main__': unittest.main()
