"""Observation gate for future visual docking; no detector or motor controller.

All tolerances are caller supplied. Robot-body pose is not accepted as basket pose
without an external measured transform. Timestamps must use the receiver's
monotonic clock domain. Camera reference validity must be checked by the adapter.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Limits:
    xy: float
    yaw: float
    age: float
    hold: float
    speed: float

    def __post_init__(self):
        if any(not math.isfinite(v) or v <= 0 for v in (self.xy,self.yaw,self.age,self.hold,self.speed)):
            raise ValueError('All limits must be supplied as positive finite values')


@dataclass(frozen=True)
class BasketObservation:
    x: float
    y: float
    yaw: float
    speed: float
    measured_at: float
    reference_id: str
    valid: bool
    reference_valid: bool


class DockingGate:
    def __init__(self, target, reference_id, limits):
        if len(target) != 3 or not all(math.isfinite(v) for v in target) or not reference_id:
            raise ValueError('Calibrated target pose and reference identity required')
        self.target, self.reference_id, self.limits = tuple(target),reference_id,limits
        self.since = None
        self.last = None

    def update(self, observation, now):
        def stop(reason):
            self.since = None
            self.last = None
            return {'state':'stopped','ready':False,'motion_allowed':False,'reason':reason}
        if observation is None:
            return stop('observation_missing')
        o = observation
        if not all(math.isfinite(v) for v in (o.x,o.y,o.yaw,o.speed,o.measured_at,now)):
            return stop('nonfinite_observation')
        if o.speed < 0:
            return stop('invalid_speed_magnitude')
        if o.valid is not True:
            return stop('tracking_invalid')
        if o.reference_valid is not True or o.reference_id != self.reference_id:
            return stop('camera_reference_changed_or_unverified')
        if not 0 <= now-o.measured_at <= self.limits.age:
            return stop('stale_or_future_observation')
        if self.last is not None and o.measured_at < self.last:
            return stop('observation_time_reversed')
        if self.last is not None and o.measured_at-self.last > self.limits.age:
            self.since = None
        fresh_frame = self.last is None or o.measured_at > self.last
        self.last = o.measured_at
        error_xy = math.hypot(o.x-self.target[0],o.y-self.target[1])
        delta = o.yaw-self.target[2]
        error_yaw = abs(math.atan2(math.sin(delta),math.cos(delta)))
        inside = error_xy <= self.limits.xy and error_yaw <= self.limits.yaw
        stopped = 0 <= o.speed <= self.limits.speed
        if not inside or not stopped:
            self.since = None
            return {'state':'aligning' if not inside else 'settling','ready':False,
                    'motion_allowed':not inside,'xy_error':error_xy,'yaw_error':error_yaw}
        if self.since is None:
            self.since = o.measured_at
        ready = fresh_frame and o.measured_at-self.since >= self.limits.hold
        return {'state':'ready' if ready else 'settling','ready':ready,'motion_allowed':False,
                'xy_error':error_xy,'yaw_error':error_yaw}
