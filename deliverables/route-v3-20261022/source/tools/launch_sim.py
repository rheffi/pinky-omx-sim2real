"""Run existing Isaac in an isolated process with local caches and a time bound."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--ros',action='store_true')
parser.add_argument('--drive-damping',type=float)
parser.add_argument('--without-basket',action='store_true')
parser.add_argument('--basket-sweep',action='store_true',help='Controlled basket-model comparison on a diagnostic floor')
parser.add_argument('--reference-media',action='store_true',help='Render assigned v3 reference poses; no physics claim')
parser.add_argument('--preview-only',action='store_true')
parser.add_argument('--capture-video',action='store_true',help='Capture the physical wheel smoke with zero-time rendering')
parser.add_argument('--timeout',type=int,default=360)
args = parser.parse_args()
if args.basket_sweep and (args.ros or args.without_basket or args.drive_damping is not None):
    parser.error('basket-sweep controls its variants and cannot be combined with smoke options')
if args.reference_media and (args.ros or args.basket_sweep or args.capture_video or args.without_basket or args.drive_damping is not None):
    parser.error('reference-media cannot be combined with physics or ROS options')
if args.preview_only and not args.reference_media:parser.error('preview-only requires reference-media')
if args.capture_video and args.basket_sweep:parser.error('capture-video applies to the wheel smoke')
output = args.output.resolve()
if output.exists():
    raise SystemExit('Choose a new output directory; evidence is never overwritten')
output.parent.mkdir(parents=True,exist_ok=True)
cache = ROOT/'.runtime'
for name in ('optix','warp','kit','data','logs','roslogs'):
    (cache/name).mkdir(parents=True,exist_ok=True)
prefix = Path('C:/isaacsim/exts/isaacsim.ros2.core/jazzy')
env = os.environ.copy()
env.update(ROS_DISTRO='jazzy',RMW_IMPLEMENTATION='rmw_fastrtps_cpp',ROS_DOMAIN_ID='83',
           ROS_LOCALHOST_ONLY='1',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
           FASTDDS_BUILTIN_TRANSPORTS='UDPv4',
           AMENT_PREFIX_PATH=str(prefix),OPTIX_CACHE_PATH=str(cache/'optix'),
           WARP_CACHE_PATH=str(cache/'warp'),PYTHONIOENCODING='utf-8',PYTHONDONTWRITEBYTECODE='1',
           ROS_LOG_DIR=str(cache/'roslogs'),
           PATH=str(prefix/'lib')+os.pathsep+env.get('PATH',''))
script = 'render_route_media.py' if args.reference_media else ('basket_sweep.py' if args.basket_sweep else 'sim_smoke.py')
command = ['C:/isaacsim/python.bat','-B',str(ROOT/'tools'/script),'--output',str(output)]
if args.ros:
    command.append('--ros')
if args.drive_damping is not None:
    command.extend(['--drive-damping',str(args.drive_damping)])
if args.without_basket:
    command.append('--without-basket')
if args.preview_only:command.append('--preview-only')
if args.capture_video:command.append('--capture-video')
started = time.monotonic()
log = output.parent/(output.name+'-console.log')
with log.open('wb') as stream:
    proc = subprocess.Popen(command,cwd=ROOT,env=env,stdout=stream,stderr=subprocess.STDOUT,
                            creationflags=subprocess.CREATE_NO_WINDOW)
    try:
        code = proc.wait(timeout=args.timeout)
        status = 'exited'
    except subprocess.TimeoutExpired:
        # Only this child's process tree; no user simulator or filesystem changes.
        subprocess.run(['taskkill','/PID',str(proc.pid),'/T','/F'],capture_output=True,timeout=15)
        proc.wait(timeout=15)
        code,status = -1,'timeout'
run_status = None
if (output/'run.json').is_file():
    run_status = json.loads((output/'run.json').read_text(encoding='utf-8')).get('status')
result = {'status':status,'exit_code':code,'run_status':run_status,'elapsed_seconds':time.monotonic()-started,
          'log':str(log),'run_directory':str(output),'time_limit_seconds':args.timeout,
          'domain':83,'network_scope':'localhost','hardware_access':False}
(output.parent/(output.name+'-supervisor.json')).write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result))
# Kit fast shutdown may return process exit 0 despite failed application checks.
expected_status = 'completed_media' if args.reference_media else ('completed_experiment' if args.basket_sweep else 'passed_smoke')
raise SystemExit(0 if code == 0 and run_status == expected_status else 1)
