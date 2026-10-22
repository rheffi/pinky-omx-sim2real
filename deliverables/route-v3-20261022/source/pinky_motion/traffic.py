"""Single-owner reservation prototype. All observations must come from an adapter.

Thread-safe atomic resource bundles, conflict-aware FIFO, and retained occupancy
on disconnect. This is not wired to cartrina-control or a physical robot.
"""
from dataclasses import dataclass
from threading import RLock
import math


@dataclass(frozen=True)
class Request:
    command_id: str
    robot_id: str
    resources: frozenset[str]


class Reservations:
    def __init__(self, resource_names):
        self.known = frozenset(resource_names)
        self.owners = {}
        self.queue = []
        self.requests = {}
        self.active = {}
        self.uncertain = set()
        self._lock = RLock()

    def request(self, command_id, robot_id, resources):
        request = Request(command_id, robot_id, frozenset(resources))
        if not command_id or not robot_id or not request.resources or not request.resources <= self.known:
            raise ValueError('Unknown or empty reservation')
        with self._lock:
            if command_id in self.requests:
                if self.requests[command_id] != request:
                    raise ValueError('Command identity reused with different contents')
                return command_id in self.active
            # One outstanding bundle per robot prevents hidden overlapping releases.
            if any(r.robot_id == robot_id for r in self.queue + list(self.active.values())):
                raise ValueError('Robot already has an outstanding reservation')
            self.requests[command_id] = request
            self.queue.append(request)
            self._grant()
            return command_id in self.active

    def _grant(self):
        blocked = set()
        for request in list(self.queue):
            if (request.robot_id in self.uncertain or request.resources & blocked
                    or any(resource in self.owners for resource in request.resources)):
                blocked.update(request.resources)
                continue
            for resource in request.resources:
                self.owners[resource] = request.robot_id
            self.active[request.command_id] = request
            self.queue.remove(request)

    def disconnect(self, robot_id):
        with self._lock:
            self.uncertain.add(robot_id)

    def release(self, command_id, robot_id, *, basket_clear, observation_fresh, identity_matches):
        # No lease timeout frees a physically occupied path. A returning robot
        # needs a fresh explicit exit observation before its reservation is freed.
        with self._lock:
            request = self.active.get(command_id)
            if not request or request.robot_id != robot_id:
                return False
            if not all(value is True for value in (basket_clear, observation_fresh, identity_matches)):
                return False
            for resource in request.resources:
                del self.owners[resource]
            del self.active[command_id]
            self.uncertain.discard(robot_id)
            self._grant()
            return True

    def cancel_waiting(self, command_id):
        with self._lock:
            before = len(self.queue)
            self.queue = [r for r in self.queue if r.command_id != command_id]
            self._grant()
            return before != len(self.queue)


class VelocityWatchdog:
    """Monotonic receiver-time watchdog; call every simulation/control step."""
    def __init__(self, timeout=.3, max_linear=.15, max_angular=1.0):
        self.timeout, self.max_linear, self.max_angular = timeout, max_linear, max_angular
        self.received = None
        self.velocity = (0.0, 0.0)

    def receive(self, linear, angular, now):
        if not all(math.isfinite(v) for v in (linear, angular, now)):
            self.received = None
            self.velocity = (0.0, 0.0)
            return False
        self.received = now
        self.velocity = (max(-self.max_linear, min(self.max_linear, linear)),
                         max(-self.max_angular, min(self.max_angular, angular)))
        return True

    def command(self, now):
        if self.received is None or not math.isfinite(now) or not 0 <= now-self.received <= self.timeout:
            return 0.0, 0.0
        return self.velocity
