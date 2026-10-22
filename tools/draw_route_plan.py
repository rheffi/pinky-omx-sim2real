"""Draw connected v3 paths with a lower-left home queue, not physical tracking."""
import argparse
from html import escape
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from pinky_motion.geometry import footprint, dock_pose, transform, valid_pose
from pinky_motion.routes import loop_samples, graph
from pinky_motion.route_paths import (station_paths,home_paths,UPPER,LOWER,CY,RADIUS,
                                      LEFT,RIGHT,ROUTE_VERSION)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--evidence', type=Path, required=True)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    cfg = json.loads((ROOT/'config/layout.json').read_text(encoding='utf-8'))
    evidence=json.loads(args.evidence.read_text(encoding='utf-8'))
    if evidence['config']!=cfg or not evidence['passed'] or evidence['route_version']!=ROUTE_VERSION:
        raise ValueError('Passing evidence matching this layout is required')
    route = graph(cfg)
    poses = loop_samples()
    scale, left, top = 600, 100, 200
    colors = dict(text='#17283b', muted='#526577', road='#087d88', entry='#356ad3',
                  exit='#bd591f', basket='#2b936f', rotation='#b58a19', wall='#455262',home='#8745b5')
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="1480" height="1450" viewBox="0 0 1480 1450">',
             '<title>200×150cm 작업장 도로 v3와 좌하단 기본 대기줄</title>',
             '<desc>차로 중심 간격 24cm. 좌하단 두 대의 일렬 대기와 진출입. 기하·운동학 모사이며 실제 물리 주행 아님.</desc>',
             '<rect width="1480" height="1450" fill="#ffffff"/>',
             '<defs>']
    for name in ('road','entry','exit','home'):
        parts.append(f'<marker id="arrow-{name}" viewBox="0 0 10 10" refX="8" refY="5" markerWidth="5" markerHeight="5" orient="auto-start-reverse"><path d="M0 0 L10 5 L0 10 Z" fill="{colors[name]}"/></marker>')
    parts.append('</defs><g font-family="Malgun Gothic, sans-serif" fill="#17283b">')

    def xy(point):
        return left + point[0]*scale, top-point[1]*scale

    def text(x, y, value, size=24, anchor='start', color=None, weight='normal'):
        parts.append(f'<text x="{x:.2f}" y="{y:.2f}" font-size="{size}" text-anchor="{anchor}" fill="{color or colors["text"]}" font-weight="{weight}">{escape(value)}</text>')

    def path(points, color, width=3, dash=None, arrow=None, opacity=1):
        d='M '+' L '.join(f'{xy(q)[0]:.3f} {xy(q)[1]:.3f}' for q in points)
        extra=(f' stroke-dasharray="{dash}"' if dash else '')+(f' marker-end="url(#arrow-{arrow})"' if arrow else '')
        parts.append(f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{width}" stroke-linejoin="round" opacity="{opacity}"{extra}/>')

    def rect(x, y, w, h, fill, stroke='none', width=1):
        px,py=xy((x,y))
        parts.append(f'<rect x="{px:.2f}" y="{py:.2f}" width="{w*scale:.2f}" height="{h*scale:.2f}" fill="{fill}" stroke="{stroke}" stroke-width="{width}"/>')

    def poly(points, fill, stroke='none', width=1):
        coords=' '.join(f'{xy(q)[0]:.3f},{xy(q)[1]:.3f}' for q in points)
        parts.append(f'<polygon points="{coords}" fill="{fill}" stroke="{stroke}" stroke-width="{width}"/>')

    def circle(point, r, fill, stroke, width=2, dash=None):
        px,py=xy(point)
        extra=f' stroke-dasharray="{dash}"' if dash else ''
        parts.append(f'<circle cx="{px:.2f}" cy="{py:.2f}" r="{r*scale:.2f}" fill="{fill}" stroke="{stroke}" stroke-width="{width}"{extra}/>')

    text(100,60,'도로 v3 · 좌하단 기본 대기와 교대 인계',40,weight='bold')
    text(100,105,'전체 200 × 150cm  |  차로 중심 간격 24cm  /  기본 대기 → 적재 → 하역 → 복귀',24,color=colors['muted'])
    text(100,145,'바구니: 살짝 띄움 가정 · 바닥 저항 제외 · 충돌 외곽은 유지',23,color=colors['muted'])
    rect(0,0,2,.75,'#f4f6f8')
    rect(0,-.75,2,.75,'#ffffff')
    # Geometry sweep is a group union visually, not a certified corridor width.
    parts.append('<g opacity="0.12">')
    for pose in poses:
        poly(footprint(cfg,pose),colors['road'])
    parts.append('</g>')
    radius=max(math.hypot(x,y) for x,y in footprint(cfg,(0,0,0)))
    for x in (.3,.9,1.6):
        circle((x,-.79),radius,'#f5ebcf',colors['rotation'],2,'7 7')
    path([(0,-.75),(2,-.75)],'#b9c3cd',2,'5 8')
    path([(0,0),(2,0),(2,-1.5),(0,-1.5),(0,0)],colors['wall'],3)
    for x in (.6,1.2):
        path([(x,0),(x,-.75)],colors['wall'],5)

    # Exact polyline from loop_samples(). Direction arrows use segments of it.
    path([(x,y) for x,y,yaw in poses],colors['road'],5)
    for a,b in [((.52,LOWER),(.70,LOWER)),((1.05,LOWER),(1.25,LOWER)),
                ((1.40,-.95),(1.23,-.95)),((.78,-.95),(.60,-.95))]:
        path([a,b],colors['road'],5,arrow='road')
    for index in (145,325):
        path([(p[0],p[1]) for p in poses[index:index+8]],colors['road'],5,arrow='road')

    branch_evidence=[]
    for i,(key,label,x) in enumerate(zip(('A','B','handoff'),('적재 A','적재 B','인계 / 하역'),(.3,.9,1.6))):
        px,_=xy((x,0))
        text(px,top+36,f'{label} · {int(cfg["station_widths"][i]*100)}cm',27,'middle',weight='bold')
        # Arm axis: actual dimension; shelf footprint deliberately not invented here.
        circle((x,-.14),.024,'#dde3e9',colors['wall'],2)
        text(px+25,top+.14*scale+8,'OMX',19)
        pose=dock_pose(cfg,i)
        rear,front=cfg['sim_asset']['rear_x'],cfg['sim_asset']['front_x']
        bw,bl=cfg['basket_width'],cfg['basket_length']
        poly(transform([(rear-bl,-bw/2),(rear,-bw/2),(rear,bw/2),(rear-bl,bw/2)],pose),'#cfe9dd',colors['basket'],2)
        poly(transform([(rear,-.055),(front,-.055),(front,.055),(rear,.055)],pose),'#dce6f7',colors['entry'],2)
        path([(x-.045,pose[1]-.015),(x+.045,pose[1]-.015)],colors['entry'],2)
        path([(x,pose[1]-.015),(x,pose[1]-.06)],colors['entry'],2,arrow='entry')
        d=cfg['junction_wall_distances'][i]
        path([(x-.08,-d),(x+.08,-d)],colors['rotation'],4)
        text(px+52,top+d*scale+8,f'접합부 {round(d*100)}cm',21)
        text(px,top+.25*scale,'적재 완료 후 순서 대기' if i<2 else '인계 1대 전용',20,'middle',colors['muted'])
        branch=station_paths(cfg,key)
        entry=branch['entry'][0].poses
        exiting=branch['exit'][1].poses
        path([(p[0],p[1]) for p in entry],colors['entry'],5,arrow='entry')
        path([(p[0],p[1]) for p in exiting],colors['exit'],5,arrow='exit')
        # Alternating colors denote the physically shared final docking neck.
        neck=[(p[0],p[1]) for p in branch['entry'][2].poses]
        path(neck,colors['entry'],5)
        path(neck,colors['exit'],5,'9 9')
        for point,color,label_,dx,dy in [(entry[0],'entry','진입',16,-16),
                                         (exiting[-1],'exit','합류',-16,30)]:
            circle(point,.010,colors[color],colors[color],1)
            nx,ny=xy(point)
            text(nx+dx,ny+dy,label_,22,'start' if dx>0 else 'end',colors[color])
        cx,cy=xy(branch['align_pose'])
        text(cx,cy-33,'180° 정렬',22,'middle')
        branch_evidence.append({'station':key,'validation':'conservative geometric sweep, 20mm per-robot margin',
                                'entry_pose':entry[0],'merge_pose':exiting[-1],
                                'shared_neck_exclusive':True,'wait_pose':branch['wait_pose'] if i<2 else None})
    # In-place rotation envelope and road-turn centre radius are different.
    for x in (LEFT,RIGHT):
        circle((x,CY),.007,colors['road'],colors['road'],1)
    path([(LEFT,CY),(LEFT-RADIUS,CY)],colors['road'],2)
    text(*xy((.47,-1.075)),'회차 R12cm',19,color=colors['road'])
    text(*xy((1.09,-1.07)),'차로 중심 간격 24cm',22,anchor='middle')
    text(*xy((1.02,-1.16)),'아래 차로 →',21,anchor='middle',color=colors['road'])
    text(*xy((1.0,-.90)),'← 위 차로',22,anchor='middle',color=colors['road'])
    home=home_paths()
    home_polys=[footprint(cfg,home[slot],.02) for slot in ('back','front')]
    hp=[p for polygon in home_polys for p in polygon]
    rect(min(x for x,y in hp),max(y for x,y in hp),
         max(x for x,y in hp)-min(x for x,y in hp),max(y for x,y in hp)-min(y for x,y in hp),
         '#f3eafa',colors['home'],2)
    for segments in (home['arrival'],home['departure']):
        for segment in segments:
            path(segment.poses,colors['home'],4,arrow='home' if segment==segments[-1] else None)
    path(home['advance'].poses,colors['home'],3,'5 5',arrow='home')
    for slot,label in [('back','후열'),('front','선두')]:
        poly(footprint(cfg,home[slot]),'#f7effc',colors['home'],2)
        p=home[slot]
        text(*xy((p[0]-.095,p[1]+.09)),label,18,anchor='middle',color=colors['home'])
    text(*xy((1.32,-1.43)),'기본 대기 2대 · 오른쪽부터 출발',21,anchor='middle',color=colors['home'])
    # Dimensions outside the floor.
    for a,b,lab in [((2.08,0),(2.08,-.75),'75cm'),((2.08,-.75),(2.08,-1.5),'75cm')]:
        path([a,b],colors['muted'],2)
        for q in (a,b): path([(q[0]-.015,q[1]),(q[0]+.015,q[1])],colors['muted'],2)
        px,py=xy((a[0]+.03,(a[1]+b[1])/2))
        text(px,py,lab,24)
    path([(0,-1.58),(2,-1.58)],colors['muted'],2)
    for x in (0,2): path([(x,-1.565),(x,-1.595)],colors['muted'],2)
    text(*xy((1,-1.635)),'전체 폭 200cm',25,anchor='middle')
    text(100,1220,'청록: 본선  /  파랑: 작업대 진입  /  주황: 출차  /  보라: 기본 대기줄',25,color=colors['road'])
    text(100,1265,'기본 대기는 좌하단. 적재 후 인계 순서를 기다릴 때는 A/B 내부를 점유.',25)
    text(100,1310,'회전 외곽 Ø47.9cm · 합류 시간 조정 · 검사 여유 2cm는 실물 보증 아님.',25)
    text(100,1355,'경로·시간 모사 검증까지 완료. Isaac 바퀴 추종·센서 도킹·실기는 별도 검증.',25)
    text(100,1400,'기준: config/layout.json + pinky_motion/routes.py  |  현장 배치 좌표계',21,color=colors['muted'])
    parts.append('</g></svg>')
    (args.output/'route-plan.svg').write_text('\n'.join(parts),encoding='utf-8')
    data={'kind':'connected_geometric_reference_not_hardware_route','route_version':ROUTE_VERSION,'geometry_source':'config/layout.json',
          'loop_source':'pinky_motion.routes.loop_samples','loop_samples':len(poses),
          'loop_samples_valid':all(valid_pose(cfg,p) for p in poses),
          'in_place_rotation_diameter_m':2*radius,'lane_centres_wall_distance_m':[-UPPER,-LOWER],
          'loop_turn_centre_radius_m':RADIUS,'station_branches':branch_evidence,
          'home_queue':{'front':home['front'],'back':home['back'],'rule':'front departs first; rear advances; return enters rear'},
          'parking_footprints':'target illustrations, not three robots simultaneously',
          'not_drawn':['unknown real shelf footprints; assumed shelf boxes included in validation','unmeasured camera FOV'],
          'basket_floor_contact':cfg['assumptions']['basket_floor_contact'],
          'kinematic_exchange_validated':True,'physical_route_tracking_validated':False,
          'evidence':str(args.evidence.resolve())}
    (args.output/'route-plan.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'svg':str(args.output/'route-plan.svg'),'loop_samples_valid':data['loop_samples_valid']}))


if __name__=='__main__':
    main()
