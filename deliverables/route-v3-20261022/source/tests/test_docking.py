from dataclasses import replace
import math
import unittest
from pinky_motion.docking import BasketObservation, DockingGate, Limits


class DockingTests(unittest.TestCase):
    def setUp(self):
        # Synthetic test tolerances only, not robot-arm acceptance specifications.
        self.gate = DockingGate((0,0,math.pi),'fixture_v1',Limits(.01,.05,.2,.3,.005))
        self.obs = BasketObservation(0,0,-math.pi,0,0,'fixture_v1',True,True)

    def tick(self, t, **changes):
        return self.gate.update(replace(self.obs,measured_at=t,**changes),t)

    def test_requires_multiple_fresh_stationary_frames(self):
        self.assertFalse(self.tick(0)['ready'])
        self.assertFalse(self.tick(.15)['ready'])
        self.assertTrue(self.tick(.31)['ready'])
        self.assertFalse(self.gate.update(replace(self.obs,measured_at=.31),.32)['ready'])

    def test_reference_change_stops_even_after_ready(self):
        for t in (0,.15,.31): self.tick(t)
        decision = self.tick(.4,reference_id='fixture_v2')
        self.assertEqual(decision['state'],'stopped')
        self.assertFalse(decision['motion_allowed'])

    def test_missing_stale_future_and_invalid_stop(self):
        for observation,now in [(None,0),(self.obs,1),(self.obs,-1),
                                (replace(self.obs,valid=False),0),
                                (replace(self.obs,reference_valid=False),0),
                                (replace(self.obs,speed=-.1),0),
                                (replace(self.obs,x=float('nan')),0)]:
            self.assertEqual(self.gate.update(observation,now)['state'],'stopped')

    def test_motion_and_error_reset_dwell(self):
        self.tick(0)
        self.tick(.15)
        self.assertFalse(self.tick(.31,speed=.03)['ready'])
        self.assertFalse(self.tick(.4)['ready'])
        self.assertEqual(self.tick(.5,x=.03)['state'],'aligning')

    def test_large_frame_gap_cannot_count_as_dwell(self):
        self.tick(0)
        self.assertFalse(self.tick(1)['ready'])

    def test_zero_tolerance_configuration_rejected(self):
        with self.assertRaises(ValueError):
            Limits(0,.1,.2,.3,.1)


if __name__ == '__main__':
    unittest.main()
