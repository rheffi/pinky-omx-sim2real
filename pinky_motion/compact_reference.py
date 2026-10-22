"""Two-order review experiment using one global movement permit.

Synthetic arm timers and ground-truth arrivals are explicitly not sensor evidence.
This is an offline review harness, not an adapter or production traffic service.
"""
import math

from . import compact_routes, route_paths
from .fleet_reference import Plan, TimedPath, plans_clear
from .geometry import overlaps
from .route_validation import valid_polygon, check_path


def order_visits(stations):
    if not stations or set(stations) - {'A', 'B'}:
        raise ValueError('An order must contain A and/or B loading work')
    return [s for s in ('A', 'B') if s in stations] + ['handoff']


def run_pipeline(config, version=4, orders=(('A', 'B'), ('A', 'B')),
                 fault=False, dt=.025, margin=.02, load_seconds=12., unload_seconds=45.):
    if version not in (3, 4) or len(orders) != 2:
        raise ValueError('Two orders and route version 3 or 4 required')
    if not all(math.isfinite(v) and v > 0 for v in (dt, load_seconds, unload_seconds)):
        raise ValueError('Positive finite timing parameters required')
    api = compact_routes if version == 4 else route_paths
    home = api.home_paths()
    robots = {rid: {'loc': slot, 'state': 'home_wait', 'visits': order_visits(order),
                    'next': 0, 'plan': Plan(home[slot]), 'distance_m': 0.,
                    'rotation_rad': 0., 'moving_seconds': 0.}
              for rid, slot, order in zip(('pinky1', 'pinky2'), ('front', 'back'), orders)}
    bays = {}; slots = {'front': 'pinky1', 'back': 'pinky2'}
    active = None; events = []; frames = []; cache = {}
    def event(t, name, rid, **extra):
        events.append(dict(t=round(t, 4), event=name, robot=rid, **extra))
    def frame(now):
        return {'t': round(now, 4),
                'poses': {rid: list(r['plan'].sample(now)[0]) for rid, r in robots.items()},
                'states': {rid: r['state'] for rid, r in robots.items()},
                'locations': {rid: r['loc'] for rid, r in robots.items()},
                'move_owner': active, 'bays': dict(bays), 'home_slots': dict(slots)}
    def route(source, target):
        key = (source, target)
        if key not in cache:
            if source == 'back' and target == 'front':
                segments = [home['advance']]
            elif source == 'front':
                segments = api.home_to_station(config, target)
            elif target in ('front', 'back'):
                segments = api.station_to_home(config, source, target)
            else:
                segments = api.transfer(config, source, target)
            if not check_path(config, route_paths.join(segments), margin)['valid']:
                raise RuntimeError('Invalid static route: '+str(key))
            cache[key] = TimedPath(segments)
        return cache[key]

    for tick in range(math.ceil(1200/dt)):
        now = tick*dt
        # Freshness loss interrupts an ongoing transfer and retains both bays
        # and the movement permit. Actual stopping distances are NOT simulated.
        if fault and active:
            r = robots[active]
            if r['loc'] == 'A' and r['target'] == 'B' and now >= r['plan'].start + r['plan'].path.duration/2:
                event(now, 'stale_observation_stop', active, source=r['loc'], target=r['target'])
                for robot in robots.values():
                    robot['plan'] = Plan(robot['plan'].sample(now)[0])
                    robot['state'] = 'fault_stop'
                for extra in range(round(3/dt)+1):
                    frames.append(frame(now+extra*dt))
                return {'route_version': version, 'status': 'stopped_on_stale',
                        'physical_simulation': False, 'events': events, 'frames': frames,
                        'move_owner': active, 'bay_owners': dict(bays),
                        'checks': {'permit_retained': active is not None,
                                   'both_bays_retained': bays.get('A') == bays.get('B') == active,
                                   'both_stopped': all(f['poses'] == frames[-1]['poses'] for f in frames[-round(3/dt):]),
                                   'continuous_reference_clear_until_stop': True}}
        if active and now >= robots[active]['plan'].end:
            rid = active; r = robots[rid]; source, target = r['loc'], r['target']
            r['plan'] = Plan(r['plan'].sample(now)[0])
            if source in bays and bays[source] == rid: del bays[source]
            if source in slots and slots[source] == rid: del slots[source]
            r['loc'] = target
            if target in ('front', 'back'):
                slots[target] = rid
                r['state'] = 'complete' if source == 'handoff' else 'home_wait'
            else:
                r['state'] = 'unloading' if target == 'handoff' else 'loading'
                r['work_end'] = now + (unload_seconds if target == 'handoff' else load_seconds)
            event(now, 'observed_arrival', rid, source=source, target=target, evidence='synthetic')
            active = None
        for rid, r in robots.items():
            if r['state'] in ('loading', 'unloading') and now >= r['work_end']:
                event(now, 'synthetic_arm_done', rid, station=r['loc'])
                r['next'] += 1
                r['state'] = 'ready'
        # Start only after arrival / synthetic arm completion. Downstream
        # work has priority so completed handoffs leave before new admissions.
        if active is None and now >= 2.:
            requests = []
            for rid, r in robots.items():
                if r['state'] == 'complete': continue
                if r['loc'] == 'back' and r['state'] == 'home_wait' and 'front' not in slots:
                    requests.append((0, rid, 'front'))
                elif r['loc'] == 'front' and r['state'] == 'home_wait':
                    target = r['visits'][0]
                    if target not in bays: requests.append((1, rid, target))
                elif r['state'] == 'ready':
                    if r['next'] == len(r['visits']):
                        target = 'front' if 'front' not in slots else 'back'
                        if target not in slots: requests.append((4, rid, target))
                    else:
                        target = r['visits'][r['next']]
                        if target not in bays:
                            requests.append((3 if target == 'handoff' else 2, rid, target))
            for _, rid, target in sorted(requests, reverse=True):
                r = robots[rid]
                path = route(r['loc'], target)
                other = robots['pinky2' if rid == 'pinky1' else 'pinky1']
                plan = Plan(path.poses[0], path, now)
                # A single moving robot still must avoid stationary robots.
                if not plans_clear(config, plan, other['plan'], now, margin, dt): continue
                if target not in ('front', 'back'): bays[target] = rid
                else: slots[target] = rid
                r.update(plan=plan, target=target, state='moving')
                r['distance_m'] += path.distance[-1]
                r['rotation_rad'] += path.angle[-1]
                r['moving_seconds'] += path.duration
                active = rid
                event(now, 'move_started', rid, source=r['loc'], target=target,
                      end=plan.end, distance_m=path.distance[-1])
                break
        envelopes = {rid: r['plan'].envelope(config, now, now+dt, margin) for rid, r in robots.items()}
        if any(not valid_polygon(config, p) for p in envelopes.values()):
            raise RuntimeError(f'Boundary/obstacle conflict at {now}')
        if overlaps(envelopes['pinky1'], envelopes['pinky2']):
            raise RuntimeError(f'Robot conflict at {now}')
        moving = sum(r['plan'].path is not None and r['plan'].start <= now < r['plan'].end for r in robots.values())
        if moving > 1: raise RuntimeError('Global movement permit violated')
        f = frame(now)
        # Explain idle states without replacing the simulator's control state.
        for rid, r in robots.items():
            if r['state'] == 'ready':
                dest = r['visits'][r['next']] if r['next'] < len(r['visits']) else 'home'
                f['states'][rid] = 'wait_handoff' if dest == 'handoff' and 'handoff' in bays else 'wait_move'
        frames.append(f)
        if all(r['state'] == 'complete' for r in robots.values()):
            return {'route_version': version, 'status': 'completed_reference_pipeline',
                    'physical_simulation': False, 'elapsed_sim_seconds': round(now, 3),
                    'distance_m': sum(r['distance_m'] for r in robots.values()),
                    'robots': {rid: {k: r[k] for k in ('distance_m', 'rotation_rad', 'moving_seconds')} for rid, r in robots.items()},
                    'timing_assumptions': {'load_seconds': load_seconds, 'unload_seconds': unload_seconds,
                                           'max_speed_m_s': .08, 'max_yaw_rad_s': .5, 'dt': dt},
                    'orders': orders, 'events': events, 'frames': frames,
                    'checks': {'all_home': True, 'all_bays_released': not bays,
                               'one_moving_at_a_time': True, 'continuous_reference_clear': True}}
    raise RuntimeError('Scenario did not complete within 1200 synthetic seconds')
