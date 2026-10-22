"""Two-robot kinematic reference experiment, not a physical controller.

Plans are timed geometric paths. Collision checks enclose complete time intervals
using path length and total yaw change, including every internal path vertex.
Observation freshness is synthetic here and must come from an adapter in use.
"""
from bisect import bisect_right
import math

from .geometry import dock_pose, footprint, overlaps, rectangle
from .route_paths import (angle_delta, interpolate, station_paths, transfer, join,
                          home_paths, home_to_station, station_to_home, ROUTE_VERSION)
from .route_validation import check_path, swept_box, valid_polygon
from .traffic import Reservations


class TimedPath:
    def __init__(self, segments, max_speed=.08, max_angular=.5):
        if not all(math.isfinite(v) and v>0 for v in (max_speed,max_angular)):
            raise ValueError('Invalid speed bounds')
        self.poses=join(segments)
        self.times=[0.0]; self.distance=[0.0]; self.angle=[0.0]
        for a,b in zip(self.poses,self.poses[1:]):
            d=math.dist(a[:2],b[:2]); w=abs(angle_delta(a[2],b[2]))
            self.times.append(self.times[-1]+max(d/max_speed,w/max_angular))
            self.distance.append(self.distance[-1]+d)
            self.angle.append(self.angle[-1]+w)
        self.duration=self.times[-1]

    def sample(self,t):
        if t<=0: return self.poses[0],0.0,0.0
        if t>=self.duration: return self.poses[-1],self.distance[-1],self.angle[-1]
        i=bisect_right(self.times,t)-1
        f=(t-self.times[i])/(self.times[i+1]-self.times[i])
        return (interpolate(self.poses[i],self.poses[i+1],f),
                self.distance[i]+f*(self.distance[i+1]-self.distance[i]),
                self.angle[i]+f*(self.angle[i+1]-self.angle[i]))


class Plan:
    def __init__(self, pose, path=None, start=0):
        self.fixed_pose=pose; self.path=path; self.start=start

    @property
    def end(self):
        return self.start+self.path.duration if self.path else self.start

    def sample(self,t):
        return self.path.sample(t-self.start) if self.path else (self.fixed_pose,0.0,0.0)

    def envelope(self,config,a,b,margin):
        p,d0,w0=self.sample(a); _,d1,w1=self.sample(b)
        r=max(math.hypot(x,y) for x,y in footprint(config,(0,0,0),margin))
        return footprint(config,p,margin+(d1-d0)+r*(w1-w0))


def plans_clear(config,a,b,start,margin=.02,dt=.025):
    end=max(a.end,b.end,start)
    count=max(1,math.ceil((end-start)/dt))
    for i in range(count):
        t=start+i*dt; u=min(end,t+dt)
        if overlaps(a.envelope(config,t,u,margin),b.envelope(config,t,u,margin)):
            return False
    return True


def schedule(config,path,now,other,margin=.02):
    # Test complete future paths, including both stationary terminal footprints.
    # Finite horizon: the other robot finishes within its known plan.
    limit=max(0,other.end-now)+1
    for k in range(math.ceil(limit/.25)+1):
        candidate=Plan(path.poses[0],path,now+k*.25)
        if plans_clear(config,candidate,other,now,margin): return candidate
    raise RuntimeError('No departure time found; waiting pose may obstruct exit')


def manoeuvre_zone(config,station,margin=.02):
    p=station_paths(config,station)
    points=[]
    for segment in p['entry']+p['exit']:
        for a,b in zip(segment.poses,segment.poses[1:]):
            points.extend(swept_box(config,a,b,margin))
    return rectangle(min(x for x,y in points),min(y for x,y in points),
                     max(x for x,y in points),max(y for x,y in points))


def run_exchange(config, fault=None, dt=.025, margin=.02):
    if fault not in (None,'stale_exit'): raise ValueError('Unknown fault')
    if dt<=0 or not math.isfinite(dt): raise ValueError('Invalid dt')
    home=home_paths()
    routes={}
    for station,slot in (('A','front'),('B','back')):
        for key,segments in [(('home',station),home_to_station(config,station)),
                             ((station,'handoff'),transfer(config,station,'handoff')),
                             (('handoff',slot),station_to_home(config,'handoff',slot))]:
            if not check_path(config,join(segments),margin)['valid']:
                raise RuntimeError('Static route validation failed: '+str(key))
            routes[key]=TimedPath(segments)
    advance=TimedPath([home['advance']])
    robots={rid:{'station':station,'slot':slot,'plan':Plan(home[slot]),
                 'state':'home_wait','task':'handoff_'+rid,'owned':False,'released':False}
            for rid,station,slot in (('pinky1','A','front'),('pinky2','B','back'))}
    reservation=Reservations(['handoff_bay','handoff_manoeuvre'])
    events=[]; frames=[]; zone=manoeuvre_zone(config,'handoff',margin)
    def event(t,name,rid,**extra):
        events.append(dict(t=round(t,4),event=name,robot=rid,**extra))
    started=False; requested=False
    moving_together=0
    for tick in range(math.ceil(360/dt)):
        now=tick*dt
        if not started and now>=2:
            first,second=robots['pinky1'],robots['pinky2']
            first['plan']=schedule(config,routes['home','A'],now,second['plan'],margin)
            first['state']='to_loading'
            event(now,'home_departure_scheduled','pinky1',start=first['plan'].start,end=first['plan'].end)
            second['plan']=schedule(config,advance,now,first['plan'],margin)
            second['state']='queue_advancing'
            event(now,'queue_advance_scheduled','pinky2',start=second['plan'].start,end=second['plan'].end)
            started=True
        # This example deliberately demonstrates two loaded robots requesting
        # one handoff bay together; loading is a synthetic three-second event.
        if not requested and all(r['state']=='waiting_loaded' for r in robots.values()):
            for rid,r in robots.items():
                granted=reservation.request(r['task'],rid,['handoff_bay','handoff_manoeuvre'])
                event(now,'request_handoff',rid,granted=granted)
            requested=True
        # Existing plans continue to their observed end; releases require clearance.
        for rid,r in robots.items():
            other=robots['pinky2' if rid=='pinky1' else 'pinky1']
            if r['state']=='queue_advancing' and now>=r['plan'].end:
                r['plan']=schedule(config,routes['home',r['station']],now,other['plan'],margin)
                r['state']='to_loading'
                event(now,'home_departure_scheduled',rid,start=r['plan'].start,end=r['plan'].end)
            elif r['state']=='to_loading' and now>=r['plan'].end:
                r['state']='loading'; r['load_end']=now+3.0
                event(now,'loading_arrival',rid,station=r['station'])
            elif r['state']=='loading' and now>=r['load_end']:
                r['state']='waiting_loaded'
                event(now,'loading_done',rid)
            if r['state']=='returning' and r['owned']:
                clear=not overlaps(footprint(config,r['plan'].sample(now)[0],margin),zone)
                if clear:
                    fresh=not (fault=='stale_exit' and rid=='pinky1')
                    released=reservation.release(r['task'],rid,basket_clear=True,
                                                 observation_fresh=fresh,identity_matches=True)
                    event(now,'exit_observation',rid,fresh=fresh,released=released)
                    if not fresh:
                        # Hold BOTH robots at current poses on invalid traffic evidence.
                        held={key:list(value['plan'].sample(now)[0]) for key,value in robots.items()}
                        for extra_tick in range(round(2/dt)+1):
                            frames.append({'t':now+extra_tick*dt,'poses':held,
                                           'states':{key:'fault_stop' for key in robots}})
                        return {'status':'held_on_stale_exit','expected_fault':True,
                                'route_version':ROUTE_VERSION,'physical_simulation':False,
                                'owners':dict(reservation.owners),'queue':[q.robot_id for q in reservation.queue],
                                'events':events,'frames':frames,'zone':zone,'fault_poses':held,
                                'checks':{'reservation_retained':set(reservation.owners.values())=={'pinky1'},
                                          'second_robot_not_admitted':robots['pinky2']['state']=='waiting_loaded',
                                          'collision_free_until_stop':True,'both_stationary_after_fault':True}}
                    r['owned']=False; r['released']=True
            if r['state']=='waiting_loaded' and r['task'] in reservation.active:
                r['owned']=True
                r['plan']=schedule(config,routes[r['station'],'handoff'],now,other['plan'],margin)
                r['state']='to_handoff'
                event(now,'departure_scheduled',rid,start=r['plan'].start,end=r['plan'].end)
            elif r['state']=='to_handoff' and now>=r['plan'].end:
                r['state']='unloading'; r['unload_end']=now+3.0
                event(now,'dock_arrival',rid)
            elif r['state']=='unloading' and now>=r['unload_end']:
                # Fixed arm work duration is a simulation event, not actual OMX/ACT.
                r['plan']=schedule(config,routes['handoff',r['slot']],now,other['plan'],margin)
                r['state']='returning'
                event(now,'unload_done_return_scheduled',rid,start=r['plan'].start,end=r['plan'].end)
            elif r['state']=='returning' and now>=r['plan'].end and r['released']:
                r['state']='complete'
                event(now,'home_arrival',rid,slot=r['slot'])
        envelopes={rid:r['plan'].envelope(config,now,now+dt,margin) for rid,r in robots.items()}
        if any(not valid_polygon(config,p) for p in envelopes.values()):
            raise RuntimeError(f'Boundary/obstacle envelope violation at {now}')
        if overlaps(envelopes['pinky1'],envelopes['pinky2']):
            raise RuntimeError(f'Two-robot envelope conflict at {now}')
        if all(r['plan'].start<=now<r['plan'].end for r in robots.values()): moving_together+=1
        poses={rid:list(r['plan'].sample(now)[0]) for rid,r in robots.items()}
        frames.append({'t':round(now,4),'poses':poses,'states':{rid:r['state'] for rid,r in robots.items()}})
        if all(r['state']=='complete' for r in robots.values()):
            return {'status':'completed_kinematic_exchange','physical_simulation':False,
                    'route_version':ROUTE_VERSION,'home_slots':{rid:r['slot'] for rid,r in robots.items()},
                    'margin_m':margin,'dt':dt,'elapsed_sim_seconds':now,
                    'simultaneous_movement_seconds':round(moving_together*dt,3),
                    'owners':dict(reservation.owners),'events':events,'frames':frames,'zone':zone,
                    'checks':{'all_returned_home':True,'collision_free_continuous_reference':True,
                              'all_reservations_released':not reservation.owners}}
    raise RuntimeError('Scenario did not finish within 360 simulated seconds')
