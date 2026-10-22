import json
import math
from pathlib import Path
import threading
import unittest

from pinky_motion.geometry import dock_pose, footprint, overlaps, rectangle, transform, valid_pose
from pinky_motion.routes import loop_samples
from pinky_motion.traffic import Reservations, VelocityWatchdog

CONFIG = json.loads((Path(__file__).resolve().parents[1]/'config/layout.json').read_text(encoding='utf-8'))


class TrafficTests(unittest.TestCase):
    def setUp(self):
        self.r = Reservations(['handoff','crossing','exit','A','B'])

    def release(self, cmd='c1', robot='p1', **overrides):
        evidence = dict(basket_clear=True,observation_fresh=True,identity_matches=True)
        evidence.update(overrides)
        return self.r.release(cmd,robot,**evidence)

    def test_second_robot_waits_for_basket_exit(self):
        self.assertTrue(self.r.request('c1','p1',['handoff','crossing','exit']))
        self.assertFalse(self.r.request('c2','p2',['handoff','crossing','exit']))
        self.assertFalse(self.release(basket_clear=False))
        self.assertNotIn('c2',self.r.active)
        self.assertTrue(self.release())
        self.assertIn('c2',self.r.active)

    def test_stale_or_wrong_evidence_retains_occupancy(self):
        self.r.request('c1','p1',['handoff'])
        for key in ['observation_fresh','identity_matches','basket_clear']:
            self.assertFalse(self.release(**{key:False}))
            self.assertEqual(self.r.owners['handoff'],'p1')
        self.assertFalse(self.release(robot='p2'))
        self.assertFalse(self.release(basket_clear='false'))

    def test_disconnect_does_not_free_resources(self):
        self.r.request('c1','p1',['handoff'])
        self.r.disconnect('p1')
        self.assertFalse(self.r.request('c2','p2',['handoff']))
        self.assertEqual(self.r.owners['handoff'],'p1')
        self.assertTrue(self.release())
        self.assertIn('c2',self.r.active)

    def test_bundle_is_atomic(self):
        self.r.request('c1','p1',['exit'])
        self.r.request('c2','p2',['handoff','exit'])
        self.assertNotIn('handoff',self.r.owners)

    def test_unrelated_zone_can_proceed(self):
        self.r.request('c1','p1',['handoff'])
        self.r.request('c2','p2',['handoff'])
        self.assertTrue(self.r.request('c3','p3',['A']))

    def test_conflicting_fifo_cannot_be_overtaken(self):
        self.r.request('c1','p1',['exit'])
        self.r.request('c2','p2',['exit','handoff'])
        self.assertFalse(self.r.request('c3','p3',['handoff']))
        self.release()
        self.assertIn('c2',self.r.active)
        self.assertNotIn('c3',self.r.active)

    def test_duplicate_does_not_reexecute_after_release(self):
        self.r.request('c1','p1',['handoff'])
        self.assertTrue(self.r.request('c1','p1',['handoff']))
        self.release()
        self.assertFalse(self.r.request('c1','p1',['handoff']))
        self.assertFalse(self.r.active)

    def test_identity_reuse_with_different_payload_rejected(self):
        self.r.request('c1','p1',['handoff'])
        with self.assertRaises(ValueError):
            self.r.request('c1','p2',['handoff'])

    def test_same_robot_cannot_hold_two_independent_bundles(self):
        self.r.request('c1','p1',['A'])
        with self.assertRaises(ValueError):
            self.r.request('c2','p1',['B'])

    def test_cancel_waiting_does_not_release_active_robot(self):
        self.r.request('c1','p1',['handoff'])
        self.r.request('c2','p2',['handoff'])
        self.assertFalse(self.r.cancel_waiting('c1'))
        self.assertTrue(self.r.cancel_waiting('c2'))
        self.assertEqual(self.r.owners,{'handoff':'p1'})

    def test_unknown_resource_rejected(self):
        with self.assertRaises(ValueError):
            self.r.request('c1','p1',['missing'])

    def test_concurrent_requests_have_only_one_owner(self):
        barrier = threading.Barrier(8)
        def request(i):
            barrier.wait()
            self.r.request(str(i),'p'+str(i),['handoff','crossing'])
        threads = [threading.Thread(target=request,args=(i,)) for i in range(8)]
        for t in threads: t.start()
        for t in threads: t.join()
        self.assertEqual(len(self.r.active),1)
        self.assertEqual(len(set(self.r.owners.values())),1)
        self.assertEqual(len(self.r.queue),7)


class WatchdogTests(unittest.TestCase):
    def test_initial_stale_and_clock_reversal_stop(self):
        w = VelocityWatchdog()
        self.assertEqual(w.command(0),(0,0))
        w.receive(.1,.5,10)
        self.assertEqual(w.command(10.1),(.1,.5))
        self.assertEqual(w.command(10.31),(0,0))
        self.assertEqual(w.command(9.9),(0,0))

    def test_invalid_command_erases_previous_motion(self):
        w = VelocityWatchdog()
        w.receive(.1,0,1)
        self.assertFalse(w.receive(float('nan'),0,1.1))
        self.assertEqual(w.command(1.11),(0,0))

    def test_speed_is_limited(self):
        w = VelocityWatchdog()
        w.receive(9,-9,0)
        self.assertEqual(w.command(0),(.15,-1.0))


class GeometryTests(unittest.TestCase):
    def test_target_refers_to_rear_junction(self):
        for i in range(3):
            pose = dock_pose(CONFIG,i)
            junction = transform([(CONFIG['sim_asset']['rear_x'],0)],pose)[0]
            self.assertAlmostEqual(junction[1],-CONFIG['junction_wall_distances'][i])
            self.assertTrue(valid_pose(CONFIG,pose))

    def test_body_only_would_miss_basket_collision(self):
        pose = (.3,-1.1,0)
        obstacle = rectangle(.08,-1.12,.1,-1.08)
        self.assertTrue(overlaps(footprint(CONFIG,pose),obstacle))
        body = transform([(-.076,-.055),(.042,-.055),(.042,.055),(-.076,.055)],pose)
        self.assertFalse(overlaps(body,obstacle))

    def test_partition_and_boundary_are_rejected(self):
        self.assertFalse(valid_pose(CONFIG,(.6,-.5,-math.pi/2)))
        self.assertFalse(valid_pose(CONFIG,(.05,-1.1,0)))

    def test_loop_sweep_fits_nominal_floor_with_10mm_margin(self):
        self.assertTrue(all(valid_pose(CONFIG,pose,.01) for pose in loop_samples()))

    def test_touching_polygons_conflict(self):
        self.assertTrue(overlaps(rectangle(0,0,1,1),rectangle(1,0,2,1)))


if __name__ == '__main__':
    unittest.main()
