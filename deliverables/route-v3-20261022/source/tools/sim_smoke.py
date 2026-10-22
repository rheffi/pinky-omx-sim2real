"""Bounded local physics and optional ROS loopback test; never connect to hardware.

Run with the existing Isaac python.bat. New run directory is mandatory.
Sensor truth below is explicitly labelled simulation ground truth, not AMCL/odom.
"""
from pathlib import Path
import argparse
import hashlib
import json
import math
import os
import sys
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from pinky_motion.traffic import VelocityWatchdog


def save(path, value):
    path.write_text(json.dumps(value, indent=2, ensure_ascii=False), encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--ros', action='store_true')
    parser.add_argument('--drive-damping', type=float)
    parser.add_argument('--without-basket', action='store_true', help='Diagnostic ablation, not project configuration')
    parser.add_argument('--no-ros-env', action='store_true')
    parser.add_argument('--capture-video',action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    config = json.loads((ROOT/'config/layout.json').read_text(encoding='utf-8'))
    if args.drive_damping is not None:
        if not math.isfinite(args.drive_damping) or args.drive_damping <= 0:
            raise ValueError('drive damping must be positive')
        config['assumptions']['wheel_drive_damping'] = args.drive_damping
    source = (ROOT/config['sim_asset']['source_repo']).resolve()
    asset = source/config['sim_asset']['pinky']
    report = {'status': 'starting', 'hardware_connected': False, 'nav2_executed': False,
              'physical_accuracy_validated': False, 'config': config, 'ros': {'requested': args.ros},
              'isaac_version': Path('C:/isaacsim/VERSION').read_text().strip(),
              'source_asset': str(asset), 'source_sha256': hashlib.sha256(asset.read_bytes()).hexdigest(),
              'script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'checks': {}}
    report['basket_present'] = not args.without_basket
    save(args.output/'run.json', report)
    from isaacsim import SimulationApp
    app = SimulationApp({'headless': True, 'width': 1000, 'height': 800,
                         'renderer': 'RaytracedLighting', 'extra_args': [
                             '--ext-folder', 'C:/isaacsim/extscache',
                             '--ext-folder', 'C:/isaacsim/extsDeprecated',
                             '--/app/tokens/cache=' + str(ROOT/'.runtime/kit'),
                             '--/app/tokens/data=' + str(ROOT/'.runtime/data'),
                             '--/app/tokens/logs=' + str(ROOT/'.runtime/logs'),
                             '--/log/file=' + str(args.output.resolve()/'kit.log')]},
                        experience=str(ROOT/'apps/pinky_motion.kit'))
    ros = None
    video = None
    try:
        import numpy as np
        import omni.usd
        import omni.timeline
        from pxr import UsdGeom, UsdPhysics, UsdShade, Gf
        from isaacsim.core.simulation_manager import SimulationManager as SM
        from isaacsim.core.experimental.prims import Articulation
        from isaacsim.sensors.experimental.rtx import RtxCamera, CameraSensor
        import omni.replicator.core as rep
        from PIL import Image

        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        world = UsdGeom.Xform.Define(stage, '/World')
        stage.SetDefaultPrim(world.GetPrim())

        def material(path, friction):
            mat = UsdShade.Material.Define(stage, path)
            api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
            api.CreateStaticFrictionAttr(friction)
            api.CreateDynamicFrictionAttr(friction)
            api.CreateRestitutionAttr(0.0)
            return mat

        floor_mat = material('/World/Materials/floor', config['assumptions']['floor_friction'])
        basket_mat = material('/World/Materials/basket', config['assumptions']['basket_friction'])

        def box(path, center, dims, color, collision=True, mat=None):
            cube = UsdGeom.Cube.Define(stage, path)
            cube.CreateSizeAttr(1.0)
            cube.AddTranslateOp().Set(Gf.Vec3d(*center))
            cube.AddScaleOp().Set(Gf.Vec3d(*dims))
            cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            if collision:
                UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            if mat:
                UsdShade.MaterialBindingAPI.Apply(cube.GetPrim()).Bind(mat, materialPurpose='physics')
            return cube

        box('/World/Floor', (1, -.75, -.02), (2, 1.5, .04), (.36,.40,.45), mat=floor_mat)
        ph = config['assumptions']['partition_height']
        pt = config['assumptions']['partition_thickness']
        for i, x in enumerate((.6, 1.2)):
            box('/World/Partitions/wall'+str(i), (x,-.375,ph/2), (pt,.75,ph), (.75,.75,.78))
        for i, (name, x) in enumerate(zip(('A','B','handoff'), (.3,.9,1.6))):
            distance = config['junction_wall_distances'][i]
            box('/World/Targets/'+name, (x,-distance,.002), (.127,.008,.002), (1,.8,.1), False)
            shelf = config['assumptions']['shelf_width_depth']
            box('/World/Shelves/'+name, (x,-.14,.055), (*shelf,.11), (.5,.32,.18))
            arm = UsdGeom.Xform.Define(stage, '/World/Arms/'+name)
            arm.AddTranslateOp().Set(Gf.Vec3d(x,-.14,.11))
            arm.AddRotateZOp().Set(-90)
            arm.GetPrim().GetReferences().AddReference(str(source/config['sim_asset']['omx']))
            # The arms are visual context only, with a parked source pose.
            for prim in list(stage.Traverse()):
                if prim.GetPath().HasPrefix(arm.GetPath()):
                    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                        UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
                    if prim.IsA(UsdPhysics.Joint):
                        UsdPhysics.Joint(prim).CreateJointEnabledAttr(False)

        root = UsdGeom.Xform.Define(stage, '/World/Pinky')
        root.GetPrim().GetReferences().AddReference(str(asset))
        root.AddTranslateOp().Set(Gf.Vec3d(.65,-1.12,.02))
        base_path = '/World/Pinky/Geometry/base_footprint/base_link'
        base = stage.GetPrimAtPath(base_path)
        if not base.HasAPI(UsdPhysics.ArticulationRootAPI):
            raise RuntimeError('Source articulation root differs from inspected asset')
        # Compound rigid basket colliders share the robot chassis rigid body.
        # The rear offset follows the converted bbox, pending physical measurement.
        rear = config['sim_asset']['rear_x']
        bw, bl, bh = (config[k] for k in ('basket_width','basket_length','basket_height'))
        wall = config['assumptions']['basket_wall']
        basket_x = rear-bl/2
        basket_z = -.028 + config['assumptions'].get('basket_ground_clearance', 0.0)
        basket_floor_contact = config['assumptions'].get('basket_floor_contact', True)
        basket_colliders = []
        for name, center, dims in [
            ('bottom',(basket_x,0,basket_z+wall/2),(bl,bw,wall)),
            ('back',(rear-bl+wall/2,0,basket_z+bh/2),(wall,bw,bh)),
            ('front',(rear-wall/2,0,basket_z+bh/2),(wall,bw,bh)),
            ('left',(basket_x,(bw-wall)/2,basket_z+bh/2),(bl,wall,bh)),
            ('right',(basket_x,-(bw-wall)/2,basket_z+bh/2),(bl,wall,bh))]:
            collider = box(base_path+'/Basket/'+name, center, dims, (.18,.65,.40), mat=basket_mat)
            basket_colliders.append(collider.GetPrim())
            if not basket_floor_contact:
                # User-selected simplification: exclude only this ground pair.
                # The basket still collides with walls, objects and other robots.
                UsdPhysics.FilteredPairsAPI.Apply(collider.GetPrim()).CreateFilteredPairsRel().AddTarget('/World/Floor')
        report['basket_floor_model'] = {
            'contact_enabled': basket_floor_contact,
            'nominal_clearance_m': config['assumptions'].get('basket_ground_clearance', 0.0),
            'obstacle_collisions_retained': True,
            'source': 'user instruction 2026-10-22; clearance value is an illustrative assumption',
        }
        report['checks']['basket_collision_filter'] = all(
            p.HasAPI(UsdPhysics.CollisionAPI)
            and (basket_floor_contact or [str(v) for v in UsdPhysics.FilteredPairsAPI(p).GetFilteredPairsRel().GetTargets()] == ['/World/Floor'])
            for p in basket_colliders
        )
        # Explicit approximation: add basket mass and parallel-axis terms in the
        # source chassis principal basis. Kept as a model assumption in the report.
        mass = UsdPhysics.MassAPI(base)
        old_mass = mass.GetMassAttr().Get()
        original_inertia = mass.GetDiagonalInertiaAttr().Get()
        original_axes = mass.GetPrincipalAxesAttr().Get()
        added_mass = config['assumptions']['basket_mass']
        old_com = np.array(mass.GetCenterOfMassAttr().Get())
        new_part = np.array([basket_x,0,basket_z+bh/2])
        new_com = (old_mass*old_com+added_mass*new_part)/(old_mass+added_mass)
        mass.CreateMassAttr(old_mass+added_mass)
        mass.CreateCenterOfMassAttr(Gf.Vec3f(*new_com.tolist()))
        # Recompute diagonal approximation in chassis XYZ, including box inertia.
        old_diag = np.array([.0009197333,.0010330667,.0009666667])
        add_diag = added_mass/12*np.array([bw*bw+bh*bh,bl*bl+bh*bh,bl*bl+bw*bw])
        diag = old_diag+add_diag
        for m, pos in ((old_mass,old_com),(added_mass,new_part)):
            delta = pos-new_com
            diag += m*(np.dot(delta,delta)-delta*delta)
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*diag.tolist()))
        mass.CreatePrincipalAxesAttr(Gf.Quatf(1))
        if args.without_basket:
            stage.RemovePrim(base_path+'/Basket')
            mass.CreateMassAttr(old_mass)
            mass.CreateCenterOfMassAttr(Gf.Vec3f(*old_com.tolist()))
            mass.CreateDiagonalInertiaAttr(original_inertia)
            mass.CreatePrincipalAxesAttr(original_axes)
        report['basket_inertia_model'] = 'diagonal compound approximation; off-diagonal terms omitted; not calibrated'
        for name in ('l_wheel_joint','r_wheel_joint'):
            drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath('/World/Pinky/Physics/'+name), 'angular')
            drive.CreateStiffnessAttr(0.0)
            drive.CreateDampingAttr(config['assumptions']['wheel_drive_damping'])
            drive.CreateMaxForceAttr(config['assumptions']['wheel_max_torque'])
            drive.CreateTargetVelocityAttr(0.0)

        from pxr import UsdLux
        UsdLux.DomeLight.Define(stage, '/World/Light').CreateIntensityAttr(1000)
        def camera(path, pos, target, hfov, resolution, capture=False):
            cam = UsdGeom.Camera.Define(stage,path)
            cam.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(*pos),Gf.Vec3d(*target),Gf.Vec3d(0,0,1)).GetInverse())
            w,h = resolution
            cam.CreateHorizontalApertureAttr(.2)
            cam.CreateVerticalApertureAttr(.2*h/w)
            cam.CreateFocalLengthAttr(.1/math.tan(math.radians(hfov)/2))
            cam.CreateClippingRangeAttr(Gf.Vec2f(.01,20))
            if capture:
                return CameraSensor(RtxCamera(path,tick_rate=0.0,reset_xform_op_properties=False),resolution=(h,w),annotators=['rgb'])
        for name, x in zip(('A','B','handoff'),(.3,.9,1.6)):
            camera('/World/Cameras/'+name,(x-.16,-.12,.8),(x,-.40,.05),70,(640,480))
        if args.capture_video:
            from media_capture import VideoWriter,captioned
            sensor=camera('/World/Cameras/overview',(1.05,-1.9,.85),(.74,-1.12,.04),55,(1280,640),True)
        else:
            sensor = camera('/World/Cameras/overview',(2.1,-2.7,2.4),(1,-.75,0),55,(1000,800),True)
        report['camera_claim'] = 'overview rendering and assumed station optics; no ACT processing or visual docking'
        SM.setup_simulation(dt=config['simulation']['dt'],device='cpu')
        SM.get_physics_scenes()[0].set_enabled_gpu_dynamics(False)
        robot = Articulation(base_path)
        stage.GetRootLayer().Export(str(args.output/'scene.usda'))
        rep.orchestrator.set_capture_on_play(False)
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(15):
            app.update()
        indices = robot.get_dof_indices(['l_wheel_joint','r_wheel_joint']).numpy().tolist()
        report['dof_names'] = robot.dof_names
        report['wheel_indices'] = indices
        report['dof_efforts_semantics'] = 'get_dof_efforts is commanded feedforward effort, not measured drive/contact torque'
        save(args.output/'run.json', report)
        if args.ros:
            from pinky_motion.ros_loopback import RosLoopback
            try:
                ros = RosLoopback(app, config)
                report['ros']['started'] = True
            except Exception:
                report['ros'].update(started=False,error=traceback.format_exc())

        dt = config['simulation']['dt']
        radius, track = config['sim_asset']['wheel_radius'], config['sim_asset']['contact_track']
        trajectory, phases = [], []
        def pose():
            p,q = robot.get_world_poses()
            p,q = p.numpy()[0],q.numpy()[0]
            yaw = math.atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]**2+q[3]**2))
            return [float(v) for v in p]+[yaw]
        capture_rows=[]
        if args.capture_video:
            # A zero-time capture must not add physics steps to the measured run.
            for _ in range(6):rep.orchestrator.step(rt_subframes=2,delta_time=0.0,pause_timeline=False)
            video=VideoWriter(args.output/'03-wheel-physics.mp4',30)
            report['video_claim']='Actual wheel target commands and PhysX poses; no position assignment during the run. One robot only, no full-route tracking.'
            report['checks']['capture_does_not_advance_physics']=True
        step = 0
        for name, seconds, v, w in [('settle',1,0,0),('forward',2,.1,0),('stop1',1,0,0),
                                   ('reverse',2,-.1,0),('stop2',1,0,0),('turn',2,0,.5),('stop3',1,0,0)]:
            start = pose()
            for _ in range(round(seconds/dt)):
                cmd = (v,w)
                if ros:
                    cmd = ros.exchange(v,w,pose(),robot.get_dof_positions().numpy()[0],robot.dof_names,step*dt)
                wheels = [(cmd[0]-cmd[1]*track/2)/radius,(cmd[0]+cmd[1]*track/2)/radius]
                robot.set_dof_velocity_targets(np.array(wheels),dof_indices=indices)
                SM.step(update_fabric=args.capture_video and SM.is_fabric_enabled())
                trajectory.append({'t':step*dt,'phase':name,'pose':pose(),'command':list(cmd),
                                   'wheel_targets':wheels,
                                   'dof_velocities':robot.get_dof_velocities().numpy()[0].tolist(),
                                   'dof_efforts':robot.get_dof_efforts().numpy()[0].tolist()})
                step += 1
                if args.capture_video and step%2==0:
                    before=pose();before_time=SM.get_simulation_time()
                    rep.orchestrator.step(rt_subframes=2,delta_time=0.0,pause_timeline=False)
                    after=pose()
                    if math.dist(before,after)>1e-7 or abs(SM.get_simulation_time()-before_time)>1e-8:
                        raise RuntimeError('Capture advanced physics; refusing to label this a timed smoke')
                    data,_=sensor.get_data('rgb')
                    if data is None:raise RuntimeError('Missing physical capture frame')
                    rgb=data.numpy() if hasattr(data,'numpy') else np.asarray(data)
                    label={'settle':'안정화','forward':'전진','stop1':'정지','reverse':'후진','stop2':'정지','turn':'회전','stop3':'최종 정지'}[name]
                    frame=captioned(rgb,'바구니 포함 바퀴 구동 · '+label,f'PhysX 물리 시험  |  1배속  |  {step*dt:.2f}초',
                        '실제 바퀴 속도 명령으로 이동 · 단일 로봇 기본 시험 · 두 대 도로 추종 / 실기 검증 아님')
                    video.add(frame)
                    capture_rows.append({'frame':video.count-1,'sim_t':step*dt,'phase':name,'pose':after})
                    if name=='turn' and abs(step*dt-8)<dt:frame.save(args.output/'wheel-physics.png')
            end = pose()
            phases.append({'name':name,'start':start,'end':end,'distance':math.dist(start[:2],end[:2]),
                           'yaw_change':math.atan2(math.sin(end[3]-start[3]),math.cos(end[3]-start[3]))})
            print('PHASE '+name+' '+json.dumps(phases[-1]), flush=True)
        by_name = {p['name']:p for p in phases}
        checks = report['checks']
        checks['forward_displacement'] = by_name['forward']['end'][0]-by_name['forward']['start'][0] > .12
        checks['reverse_displacement'] = by_name['reverse']['end'][0]-by_name['reverse']['start'][0] < -.12
        checks['turn_response'] = .5 < by_name['turn']['yaw_change'] < 1.5
        checks['bounded_stop_drift'] = all(by_name[n]['distance'] < .02 for n in ('stop1','stop2','stop3'))
        checks['stable_height'] = all(.015 < r['pose'][2] < .08 for r in trajectory)
        checks['finite_trajectory'] = all(all(math.isfinite(v) for v in r['pose']) for r in trajectory)
        report['phases'] = phases
        if video is not None:
            report['video']=video.close();video=None
            save(args.output/'capture-frames.json',capture_rows)
        save(args.output/'trajectory.json',trajectory)
        if ros:
            report['ros'].update(ros.finish_test())
            checks['ros_loopback'] = report['ros']['passed']
        elif args.ros:
            checks['ros_loopback'] = False
        # Rendering advances a few more zero-command steps, excluded from motion metrics.
        for _ in range(12):
            rep.orchestrator.step(rt_subframes=2,delta_time=dt,pause_timeline=False)
            data,_ = sensor.get_data('rgb')
            if data is not None:
                rgb = data.numpy() if hasattr(data,'numpy') else np.asarray(data)
                if rgb.size and np.ptp(rgb[:,:,:3]) > 10:
                    Image.fromarray(rgb[:,:,:3].astype(np.uint8)).save(args.output/'overview.png')
                    checks['rendered_overview'] = True
                    break
        report['status'] = 'passed_smoke' if checks and all(checks.values()) else 'failed_checks'
        report['claims'] = 'Local wheel actuation and rigid basket smoke only; no localization, Nav2, camera docking, two-robot physics, or hardware validation.'
    except Exception:
        report.update(status='failed',error=traceback.format_exc())
        print(report['error'],flush=True)
    finally:
        if video is not None:
            try:video.close()
            except Exception:pass
        save(args.output/'run.json',report)
        if ros:
            ros.close()
        print('RESULT '+report['status'],flush=True)
        app.close()
    return 0 if report['status'] == 'passed_smoke' else 1


if __name__ == '__main__':
    raise SystemExit(main())
