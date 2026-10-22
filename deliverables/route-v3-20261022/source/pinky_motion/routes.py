"""Directed layout-frame reference graph, not a Nav2 Route Server map."""
import math
from .geometry import dock_pose
from .route_paths import (STATIONS, station_paths, loop_leg, loop_pose, home_paths,
                          LEFT, RIGHT, CY, RADIUS, ROUTE_VERSION, Segment, join)


def loop_samples(step=.01):
    points = []
    left, right, cy, radius = LEFT,RIGHT,CY,RADIUS
    count = math.ceil((right-left)/step)
    for i in range(count+1):
        points.append((left+(right-left)*i/count,cy-radius,0.0))
    count_arc = math.ceil(math.pi*radius/step)
    for i in range(1,count_arc+1):
        a = -math.pi/2+math.pi*i/count_arc
        points.append((right+radius*math.cos(a),cy+radius*math.sin(a),a+math.pi/2))
    for i in range(1,count+1):
        points.append((right-(right-left)*i/count,cy+radius,math.pi))
    for i in range(1,count_arc+1):
        a = math.pi/2+math.pi*i/count_arc
        points.append((left+radius*math.cos(a),cy+radius*math.sin(a),a+math.pi/2))
    return points


def graph(config):
    nodes,edges,road_nodes={},[],[]
    def edge(a,b,segment,resources):
        edges.append({'from':a,'to':b,'direction':segment.direction,
                      'reference_poses':segment.poses,'required_resources':resources,
                      'validation':'geometric_reference; not physical tracking'})
    for name in STATIONS:
        p=station_paths(config,name)
        for suffix,s in [('entry',p['entry_s']),('merge',p['exit_s'])]:
            nodes[name+'_'+suffix]={'pose':loop_pose(s),'role':suffix}
            road_nodes.append((s,name+'_'+suffix))
        nodes[name+'_align_north']={'pose':p['entry'][0].poses[-1],'role':'align'}
        nodes[name+'_align_south']={'pose':p['entry'][1].poses[-1],'role':'align'}
        nodes[name+'_dock']={'pose':p['wait_pose'],'role':'loading_wait' if name!='handoff' else 'handoff'}
        resources=[name+'_bay',name+'_manoeuvre']
        for a,b,segment in [('_entry','_align_north',p['entry'][0]),
                            ('_align_north','_align_south',p['entry'][1]),
                            ('_align_south','_dock',p['entry'][2]),
                            ('_dock','_align_south',p['exit'][0]),
                            ('_align_south','_merge',p['exit'][1])]:
            edge(name+a,name+b,segment,resources)
    home=home_paths()
    for suffix,s in [('entry',home['entry_s']),('merge',home['exit_s'])]:
        nodes['home_'+suffix]={'pose':loop_pose(s),'role':'home_'+suffix}
        road_nodes.append((s,'home_'+suffix))
    for slot in ('back','front'):
        nodes['home_'+slot]={'pose':home[slot],'role':'home_queue_'+slot}
    edge('home_entry','home_back',Segment('home_arrival',join(home['arrival']),1),['home_queue_entry'])
    edge('home_back','home_front',home['advance'],['home_queue'])
    edge('home_front','home_merge',Segment('home_departure',join(home['departure']),1),['home_queue','home_merge'])
    road_nodes.sort()
    for (s,a),(t,b) in zip(road_nodes,road_nodes[1:]+road_nodes[:1]):
        edge(a,b,loop_leg(s,t),['road_'+a+'_'+b])
    return {'schema_version':2,'route_version':ROUTE_VERSION,'kind':'reference_graph_not_nav2_serialization',
            'frame':'layout (not pinky_map_new map)','nodes':nodes,'edges':edges,
            'notes':['Complete geometric connections with explicit yaw and reverse segments.',
                     'Road resource names alone do not encode cross-edge footprint conflicts.',
                     'Fleet experiment additionally checks whole timed footprint envelopes.',
                     'Home queue at lower left; front departs first, rear advances when clear.',
                     'Loading A/B remain occupied while loaded robots wait for handoff.',
                     'No real camera or physical route tracking verified by this graph.']}
