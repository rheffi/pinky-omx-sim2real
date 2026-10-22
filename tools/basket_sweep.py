"""Controlled multi-model basket ablation on a diagnostic test floor.

This floor is NOT the shopping layout. All models receive the same command and
physics steps, with separate starting pads. Parameter choices are assumptions.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[1]


def save(path, value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    config = json.loads((ROOT/'config/layout.json').read_text(encoding='utf-8'))
    asset = (ROOT/config['sim_asset']['source_repo']/config['sim_asset']['pinky']).resolve()
    variants = [
        dict(name='bare',mass=0,contact=False,mu=.25,combine='average',lift=0),
        dict(name='baseline',mass=.05,contact=True,mu=.25,combine='average',lift=0),
        dict(name='mass_only',mass=.05,contact=False,mu=.25,combine='average',lift=0),
        dict(name='zero_friction',mass=.05,contact=True,mu=0,combine='min',lift=0),
        dict(name='low_friction',mass=.05,contact=True,mu=.05,combine='min',lift=0),
        dict(name='low_mass',mass=.005,contact=True,mu=.25,combine='average',lift=0),
        dict(name='raised_1mm',mass=.05,contact=True,mu=.25,combine='average',lift=.001),
        dict(name='contact_offset_1mm',mass=.05,contact=True,mu=.25,combine='average',lift=0,contact_offset=.001),
        dict(name='baseline_repeat',mass=.05,contact=True,mu=.25,combine='average',lift=0),
    ]
    report = {'status':'starting','kind':'controlled_basket_model_ablation','hardware_connected':False,
              'physical_accuracy_validated':False,'variants':variants,'config':config,
              'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'source_asset_sha256':hashlib.sha256(asset.read_bytes()).hexdigest(),
              'isaac_version':Path('C:/isaacsim/VERSION').read_text().strip(),
              'contact_force_semantics':'RigidPrim reported contact forces divided by physics dt; not motor torque. Base includes compound basket.',
              'reference':'https://nvidia-omniverse.github.io/PhysX/physx/latest/_api_build/structPxCombineMode.html'}
    save(args.output/'run.json',report)
    from isaacsim import SimulationApp
    app = SimulationApp({'headless':True,'width':1100,'height':1100,'renderer':'RaytracedLighting',
                         'extra_args':['--ext-folder','C:/isaacsim/extscache','--ext-folder','C:/isaacsim/extsDeprecated',
                                       '--/app/tokens/cache='+str(ROOT/'.runtime/kit'),
                                       '--/app/tokens/data='+str(ROOT/'.runtime/data'),
                                       '--/app/tokens/logs='+str(ROOT/'.runtime/logs'),
                                       '--/log/file='+str(args.output.resolve()/'kit.log')]},
                        experience=str(ROOT/'apps/pinky_motion.kit'))
    try:
        import numpy as np
        import omni.usd
        import omni.timeline
        import omni.replicator.core as rep
        from pxr import UsdGeom, UsdPhysics, UsdShade, UsdLux, PhysxSchema, Gf
        from isaacsim.core.simulation_manager import SimulationManager as SM
        from isaacsim.core.experimental.prims import Articulation, RigidPrim
        from isaacsim.sensors.experimental.rtx import RtxCamera, CameraSensor
        from PIL import Image
        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageMetersPerUnit(stage,1.0)
        UsdGeom.SetStageUpAxis(stage,UsdGeom.Tokens.z)
        stage.SetDefaultPrim(UsdGeom.Xform.Define(stage,'/World').GetPrim())

        def material(path, friction, combine='average'):
            mat = UsdShade.Material.Define(stage,path)
            api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
            api.CreateStaticFrictionAttr(friction)
            api.CreateDynamicFrictionAttr(friction)
            api.CreateRestitutionAttr(0.0)
            PhysxSchema.PhysxMaterialAPI.Apply(mat.GetPrim()).CreateFrictionCombineModeAttr(combine)
            return mat

        def box(path, center, dims, color, collision, mat=None, contact_offset=None):
            prim = UsdGeom.Cube.Define(stage,path)
            prim.CreateSizeAttr(1.0)
            prim.AddTranslateOp().Set(Gf.Vec3d(*center))
            prim.AddScaleOp().Set(Gf.Vec3d(*dims))
            prim.CreateDisplayColorAttr([Gf.Vec3f(*color)])
            if collision:
                UsdPhysics.CollisionAPI.Apply(prim.GetPrim())
                if contact_offset is not None:
                    api = PhysxSchema.PhysxCollisionAPI.Apply(prim.GetPrim())
                    api.CreateContactOffsetAttr(contact_offset)
                    api.CreateRestOffsetAttr(0.0)
            if mat:
                UsdShade.MaterialBindingAPI.Apply(prim.GetPrim()).Bind(mat,materialPurpose='physics')
            return prim

        floor_mat = material('/World/Materials/floor',.6)
        box('/World/Floor',(1.6,1.4,-.02),(3.8,3.6,.04),(.55,.58,.63),True,floor_mat)
        base_paths,contact_paths,labels = [],[],[]
        for i,variant in enumerate(variants):
            origin = [.6+(i%3),.4+(i//3),.02]
            root_path = '/World/Robots/r'+str(i)
            root = UsdGeom.Xform.Define(stage,root_path)
            root.GetPrim().GetReferences().AddReference(str(asset))
            root.AddTranslateOp().Set(Gf.Vec3d(*origin))
            base_path = root_path+'/Geometry/base_footprint/base_link'
            base = stage.GetPrimAtPath(base_path)
            base_paths.append(base_path)
            contact_paths.extend([base_path,base_path+'/l_wheel',base_path+'/r_wheel',
                                  base_path+'/caster_mount_link/caster_roate_link/caster_wheel'])
            # The same compound mass approximation used by the initial smoke.
            if variant['name'] != 'bare':
                bw,bl,bh = .127,.155,.043
                wall,rear = .003,config['sim_asset']['rear_x']
                bx,bz = rear-bl/2,-.028+variant['lift']
                mat = material('/World/Materials/basket'+str(i),variant['mu'],variant['combine'])
                variant['effective_basket_floor_mu'] = (variant['mu']+.6)/2 if variant['combine']=='average' else min(variant['mu'],.6)
                for name,center,dims in [
                    ('bottom',(bx,0,bz+wall/2),(bl,bw,wall)),
                    ('back',(rear-bl+wall/2,0,bz+bh/2),(wall,bw,bh)),
                    ('front',(rear-wall/2,0,bz+bh/2),(wall,bw,bh)),
                    ('left',(bx,(bw-wall)/2,bz+bh/2),(bl,wall,bh)),
                    ('right',(bx,-(bw-wall)/2,bz+bh/2),(bl,wall,bh))]:
                    box(base_path+'/Basket/'+name,center,dims,(.17,.65,.42),variant['contact'],mat,variant.get('contact_offset'))
                mass = UsdPhysics.MassAPI(base)
                old_m = mass.GetMassAttr().Get()
                old_com = np.array(mass.GetCenterOfMassAttr().Get())
                part = np.array([bx,0,bz+bh/2])
                added = variant['mass']
                com = (old_m*old_com+added*part)/(old_m+added)
                diag = np.array([.0009197333,.0010330667,.0009666667])+added/12*np.array([bw*bw+bh*bh,bl*bl+bh*bh,bl*bl+bw*bw])
                for m,pos in ((old_m,old_com),(added,part)):
                    delta = pos-com
                    diag += m*(np.dot(delta,delta)-delta*delta)
                mass.CreateMassAttr(old_m+added)
                mass.CreateCenterOfMassAttr(Gf.Vec3f(*com.tolist()))
                mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*diag.tolist()))
                mass.CreatePrincipalAxesAttr(Gf.Quatf(1))
            for name in ('l_wheel_joint','r_wheel_joint'):
                drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(root_path+'/Physics/'+name),'angular')
                drive.CreateStiffnessAttr(0.0)
                drive.CreateDampingAttr(.2)
                drive.CreateMaxForceAttr(.3)
                drive.CreateTargetVelocityAttr(0.0)
            labels.append({'index':i,'name':variant['name'],'origin':origin})
            box('/World/Pads/pad'+str(i),(origin[0],origin[1],.001),(.86,.86,.001),(.65,.68,.72),False)

        UsdLux.DomeLight.Define(stage,'/World/Light').CreateIntensityAttr(1100)
        cam = UsdGeom.Camera.Define(stage,'/World/Camera')
        cam.AddTransformOp().Set(Gf.Matrix4d().SetLookAt(Gf.Vec3d(1.6,1.4,4.5),Gf.Vec3d(1.6,1.4,0),Gf.Vec3d(0,1,0)).GetInverse())
        cam.CreateHorizontalApertureAttr(.2)
        cam.CreateVerticalApertureAttr(.2)
        cam.CreateFocalLengthAttr(.1/math.tan(math.radians(48)/2))
        cam.CreateClippingRangeAttr(Gf.Vec2f(.01,20))
        sensor = CameraSensor(RtxCamera('/World/Camera',tick_rate=0.0,reset_xform_op_properties=False),resolution=(1100,1100),annotators=['rgb'])
        dt = config['simulation']['dt']
        SM.setup_simulation(dt=dt,device='cpu')
        SM.get_physics_scenes()[0].set_enabled_gpu_dynamics(False)
        robots = Articulation(base_paths)
        contacts = RigidPrim(contact_paths,contact_filter_paths=['/World/Floor'],max_contact_count=1024)
        contacts.set_enabled_contact_tracking([True],threshold=0.0)
        stage.GetRootLayer().Export(str(args.output/'bench.usda'))
        rep.orchestrator.set_capture_on_play(False)
        omni.timeline.get_timeline_interface().play()
        for _ in range(15): app.update()
        indices = robots.get_dof_indices(['l_wheel_joint','r_wheel_joint']).numpy().tolist()
        report.update(labels=labels,wheel_indices=indices,dof_names=robots.dof_names,
                      contact_order_per_variant=['chassis_and_basket','left_wheel','right_wheel','caster_wheel'])
        rows,phases=[],[]
        def read():
            p,q = robots.get_world_poses()
            p,q = p.numpy(),q.numpy()
            yaw=np.arctan2(2*(q[:,0]*q[:,3]+q[:,1]*q[:,2]),1-2*(q[:,2]**2+q[:,3]**2))
            return np.column_stack([p,yaw])
        step=0
        sequence=[('settle',1,0,0),('forward',2,.1,0),('stop1',1,0,0),('reverse',2,-.1,0),
                  ('stop2',1,0,0),('turn_short',2,0,.5),('stop3',1,0,0),('turn_180',2*math.pi,0,.5),('stop4',1,0,0)]
        for name,seconds,v,w in sequence:
            starts=read()
            count=round(seconds/dt)
            for _ in range(count):
                wheels=[(v-w*.0971/2)/.028,(v+w*.0971/2)/.028]
                robots.set_dof_velocity_targets(np.tile(wheels,(len(variants),1)),dof_indices=indices)
                SM.step()
                rows.append({'t':step*dt,'phase':name,'pose':read().tolist(),
                             'wheel_velocities':robots.get_dof_velocities().numpy()[:,indices].tolist(),
                             'floor_contact_N':contacts.get_contact_force_matrix(dt=dt).numpy().reshape(len(variants),4,3).tolist()})
                step+=1
            ends=read()
            phases.append({'name':name,'steps':count,'duration':count*dt,'command':[v,w],
                           'start':starts.tolist(),'end':ends.tolist()})
            print('PHASE '+name,flush=True)
        metrics=[]
        for i,variant in enumerate(variants):
            result={'variant':variant['name']}
            for name in ('forward','reverse','turn_short','turn_180','stop4'):
                selected=[r for r in rows if r['phase']==name]
                phase=next(p for p in phases if p['name']==name)
                angles=np.unwrap([phase['start'][i][3]]+[r['pose'][i][3] for r in selected])
                yaw=float(angles[-1]-angles[0])
                force=np.array([r['floor_contact_N'][i] for r in selected[10:]])
                result[name]={'yaw_degrees':math.degrees(yaw),
                              'expected_yaw_degrees':math.degrees(phase['duration']*phase['command'][1]),
                              'distance_m':math.dist(phase['start'][i][:2],phase['end'][i][:2]),
                              'mean_contact_z_N':force[:,:,2].mean(axis=0).tolist(),
                              'mean_wheel_rad_s':np.array([r['wheel_velocities'][i] for r in selected[10:]]).mean(axis=0).tolist()}
            metrics.append(result)
        report.update(metrics=metrics,phases=phases,finite=all(math.isfinite(v) for row in rows for pose in row['pose'] for v in pose))
        save(args.output/'trajectory.json',rows)
        for _ in range(10):
            rep.orchestrator.step(rt_subframes=2,delta_time=dt,pause_timeline=False)
            data,_=sensor.get_data('rgb')
            if data is not None:
                rgb=data.numpy() if hasattr(data,'numpy') else np.asarray(data)
                if rgb.size and np.ptp(rgb[:,:,:3])>10:
                    Image.fromarray(rgb[:,:,:3].astype(np.uint8)).save(args.output/'overview.png')
                    report['overview']='overview.png'
                    break
        report['status']='completed_experiment' if report['finite'] else 'invalid_experiment'
        report['claim']='Sensitivity and cause isolation in assumed models, not real basket calibration or a chosen friction value.'
    except Exception:
        report.update(status='failed',error=traceback.format_exc())
        print(report['error'],flush=True)
    finally:
        save(args.output/'run.json',report)
        print('RESULT '+report['status'],flush=True)
        app.close()


if __name__=='__main__':
    main()
