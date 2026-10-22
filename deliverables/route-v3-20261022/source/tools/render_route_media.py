"""Isaac-rendered reference playback. Poses are assigned, never physics results."""
from pathlib import Path
from bisect import bisect_right
import argparse,hashlib,json,math,sys,traceback

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from pinky_motion.route_paths import station_paths,home_paths
from pinky_motion.routes import loop_samples


def save(path,data):path.write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--preview-only',action='store_true')
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    source_scene=ROOT/'outputs/smoke-005-raised-basket/scene.usda'
    evidence_dir=ROOT/'outputs/routes-v3-001'
    evidence=json.loads((evidence_dir/'result.json').read_text(encoding='utf-8'))
    if not evidence['passed'] or evidence['route_version']!=3:raise ValueError('Passing v3 evidence required')
    config=evidence['config']
    frames={key:json.loads((evidence_dir/(key+'-frames.json')).read_text(encoding='utf-8'))
            for key in ('exchange','stale_exit')}
    times={key:[r['t'] for r in rows] for key,rows in frames.items()}
    report={'status':'starting','kind':'isaac_rendered_reference_playback','route_version':3,
            'poses_assigned_from_reference':True,'physics_route_tracking':False,'hardware_connected':False,
            'source_scene':str(source_scene),'source_evidence':str(evidence_dir),
            'source_evidence_sha256':hashlib.sha256((evidence_dir/'result.json').read_bytes()).hexdigest(),
            'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'videos':[],'rendered_frames':0,'isaac_version':Path('C:/isaacsim/VERSION').read_text().strip()}
    save(args.output/'run.json',report)
    from isaacsim import SimulationApp
    app=SimulationApp({'headless':True,'width':1280,'height':640,'renderer':'RaytracedLighting',
        'extra_args':['--ext-folder','C:/isaacsim/extscache','--ext-folder','C:/isaacsim/extsDeprecated',
          '--/app/tokens/cache='+str(ROOT/'.runtime/kit'),'--/app/tokens/data='+str(ROOT/'.runtime/data'),
          '--/app/tokens/logs='+str(ROOT/'.runtime/logs'),'--/log/file='+str(args.output.resolve()/'kit.log')]},
        experience=str(ROOT/'apps/pinky_motion.kit'))
    writer=None
    try:
        import numpy as np
        import omni.usd,omni.timeline
        import omni.replicator.core as rep
        from pxr import UsdGeom,UsdPhysics,Gf
        from isaacsim.sensors.experimental.rtx import RtxCamera,CameraSensor
        from media_capture import VideoWriter,captioned
        omni.usd.get_context().new_stage()
        stage=omni.usd.get_context().get_stage()
        # Reference only world geometry: an exported /Render tree contains
        # session-specific products and must not replace Kit's active viewport.
        UsdGeom.SetStageMetersPerUnit(stage,1.0);UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
        world=UsdGeom.Xform.Define(stage,'/World');stage.SetDefaultPrim(world.GetPrim())
        world.GetPrim().GetReferences().AddReference(str(source_scene),'/World')
        second=UsdGeom.Xform.Define(stage,'/World/Pinky2')
        second.GetPrim().GetReferences().AddReference(str(source_scene),'/World/Pinky')
        for prim in list(stage.Traverse()):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
            if prim.IsA(UsdPhysics.Joint):UsdPhysics.Joint(prim).CreateJointEnabledAttr(False)
        root_ops=[]
        for name,color in [('Pinky',(.85,.16,.22)),('Pinky2',(.18,.32,.86))]:
            robot=UsdGeom.Xformable(stage.GetPrimAtPath('/World/'+name));robot.ClearXformOpOrder()
            root_ops.append((robot.AddTranslateOp(opSuffix='media'),robot.AddRotateZOp(opSuffix='media')))
            for part in ('bottom','back','front','left','right'):
                UsdGeom.Gprim(stage.GetPrimAtPath('/World/'+name+'/Geometry/base_footprint/base_link/Basket/'+part)).CreateDisplayColorAttr([Gf.Vec3f(*color)])
        # Floor stripes follow the same reference paths, with no collision shape.
        def curve(name,poses,color,width=.008):
            obj=UsdGeom.BasisCurves.Define(stage,'/World/Roads/'+name)
            obj.CreateTypeAttr('linear');obj.CreateCurveVertexCountsAttr([len(poses)])
            obj.CreatePointsAttr([Gf.Vec3f(p[0],p[1],.004) for p in poses])
            obj.CreateWidthsAttr([width]);obj.SetWidthsInterpolation('constant')
            obj.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        curve('loop',loop_samples(),(.02,.44,.49))
        for name in ('A','B','handoff'):
            branch=station_paths(config,name)
            for direction,segments,color in [('entry',branch['entry'],(.16,.36,.85)),('exit',branch['exit'],(.87,.37,.10))]:
                for i,s in enumerate(segments):
                    if s.name!='align_turn':curve(name+direction+str(i),s.poses,color,.006)
        home=home_paths()
        for i,s in enumerate(home['arrival']+[home['advance']]+home['departure']):
            curve('home'+str(i),s.poses,(.53,.18,.72),.007)
        cam=UsdGeom.Camera.Define(stage,'/World/Cameras/media')
        cam.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(1,-2.5,2.5),Gf.Vec3d(1,-.73,.02),Gf.Vec3d(0,0,1)).GetInverse())
        cam.CreateHorizontalApertureAttr(.2);cam.CreateVerticalApertureAttr(.1)
        cam.CreateFocalLengthAttr(.1/math.tan(math.radians(58)/2));cam.CreateClippingRangeAttr(Gf.Vec2f(.01,20))
        sensor=CameraSensor(RtxCamera(str(cam.GetPath()),tick_rate=0.0,reset_xform_op_properties=False),resolution=(640,1280),annotators=['rgb'])
        rep.orchestrator.set_capture_on_play(False)
        omni.timeline.get_timeline_interface().stop()
        state_labels={'home_wait':'기본 대기','queue_advancing':'대기줄 전진','to_loading':'적재 이동','loading':'적재 모사',
          'waiting_loaded':'인계 순서 대기','to_handoff':'인계 이동','unloading':'하역 모사','returning':'기본 대기 복귀','complete':'복귀 완료','fault_stop':'정지 · 점유 유지'}
        def render(key,t,title,speed):
            row=frames[key][max(0,bisect_right(times[key],t)-1)]
            for index,rid in enumerate(('pinky1','pinky2')):
                x,y,yaw=row['poses'][rid]
                root_ops[index][0].Set(Gf.Vec3d(x,y,0));root_ops[index][1].Set(math.degrees(yaw))
            rep.orchestrator.step(rt_subframes=2,delta_time=0.0,pause_timeline=True)
            data,_=sensor.get_data('rgb')
            if data is None:raise RuntimeError('No RGB frame')
            rgb=data.numpy() if hasattr(data,'numpy') else np.asarray(data)
            report['rendered_frames']+=1
            subtitle=f'기준 경로 3D 재생  |  {speed:g}배속  |  {row["t"]:.2f}초'
            footer='빨강 1번: '+state_labels[row['states']['pinky1']]+'    /    파랑 2번: '+state_labels[row['states']['pinky2']]+'    ·    바퀴 물리 추종·실기 검증 아님'
            return captioned(rgb,title,subtitle,footer)
        # Warm-up at initial poses: no physics or scene time advancement.
        for index,rid in enumerate(('pinky1','pinky2')):
            x,y,yaw=frames['exchange'][0]['poses'][rid]
            root_ops[index][0].Set(Gf.Vec3d(x,y,0));root_ops[index][1].Set(math.degrees(yaw))
        for _ in range(8):rep.orchestrator.step(rt_subframes=2,delta_time=0.0,pause_timeline=True)
        preview=render('exchange',0,'Pinky 2대 · 도로 v3',4)
        preview.save(args.output/'preview.png')
        stage.GetRootLayer().Export(str(args.output/'reference-scene.usda'))
        if not args.preview_only:
            fps=24
            clips=[('01-route-overview.mp4',[('exchange',0,times['exchange'][-1],4,'전체 운행 · 대기 → 적재 → 교대 하역 → 복귀')]),
                   ('02-key-moments.mp4',[('exchange',0,16,2,'대기줄 · 앞차 출발과 뒤차 전진'),
                     ('exchange',92,114,2,'교대 인계 · 바구니 이탈 확인 후 다음 차 진입'),
                     ('stale_exit',102,times['stale_exit'][-1],1,'이상 대응 · 오래된 이탈 정보에서 정지와 점유 유지')])]
            for filename,segments in clips:
                writer=VideoWriter(args.output/filename,fps);chapter=[]
                for key,start,end,speed,title in segments:
                    offset=writer.count/fps
                    for i in range(math.ceil((end-start)/speed*fps)+1):
                        t=min(end,start+i*speed/fps)
                        writer.add(render(key,t,title,speed))
                    chapter.append({'video_start_s':offset,'scenario':key,'source_start_s':start,'source_end_s':end,'playback_speed':speed,'title':title})
                info=writer.close();writer=None;info['chapters']=chapter;report['videos'].append(info)
                save(args.output/'run.json',report)
            # Three additional Isaac stills, supplementary to six 2D keyframes.
            for name,key,t in [('isaac-home','exchange',0),('isaac-handoff','exchange',94),('isaac-stop','stale_exit',108.4)]:
                render(key,t,'Pinky 도로 v3 · '+name,1).save(args.output/(name+'.png'))
        report['status']='completed_media'
    except Exception:
        report.update(status='failed',error=traceback.format_exc());print(report['error'],flush=True)
    finally:
        if writer is not None:
            try:writer.close()
            except Exception:pass
        save(args.output/'run.json',report);print('RESULT '+report['status'],flush=True);app.close()
    return 0 if report['status']=='completed_media' else 1


if __name__=='__main__':raise SystemExit(main())
