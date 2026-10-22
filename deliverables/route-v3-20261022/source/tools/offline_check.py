"""Save repeatable geometry/reservation evidence without Isaac or ROS."""
from pathlib import Path
import argparse
import json
import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from pinky_motion.geometry import valid_pose, footprint
from pinky_motion.routes import graph, loop_samples
from pinky_motion.traffic import Reservations

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path,required=True)
args = parser.parse_args()
args.output.mkdir(parents=True,exist_ok=False)
cfg = json.loads((ROOT/'config/layout.json').read_text(encoding='utf-8'))
samples = loop_samples()
checks = {str(margin):all(valid_pose(cfg,p,margin) for p in samples) for margin in (0,.01,.02,.03,.04)}
r = Reservations(['handoff','crossing','exit'])
trace = []
def event(name,result):
    trace.append({'event':name,'result':result,'owners':dict(r.owners),'waiting':[q.robot_id for q in r.queue]})
event('pinky1_requests_handoff',r.request('c1','pinky1',['handoff','crossing','exit']))
event('pinky2_requests_handoff',r.request('c2','pinky2',['handoff','crossing','exit']))
event('unload_done_but_basket_not_clear',r.release('c1','pinky1',basket_clear=False,observation_fresh=True,identity_matches=True))
r.disconnect('pinky1')
event('pinky1_disconnected',None)
event('fresh_observation_confirms_basket_exit',r.release('c1','pinky1',basket_clear=True,observation_fresh=True,identity_matches=True))
result = {'kind':'offline_geometry_and_synthetic_event_test','physical_simulation':False,
          'loop_sample_count':len(samples),'pose_margin_pass':checks,'reservation_trace':trace,
          'limits':['Discrete sweep, not continuous collision proof.','This legacy script does not check station manoeuvres; use tools/check_routes.py.',
                    'No localization error or braking uncertainty in this geometry check.',
                    'Exit observations in this trace are synthetic test inputs.']}
for name,data in [('result.json',result),('route-proposal.json',graph(cfg)),('loop-samples.json',samples)]:
    (args.output/name).write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result,ensure_ascii=False))
