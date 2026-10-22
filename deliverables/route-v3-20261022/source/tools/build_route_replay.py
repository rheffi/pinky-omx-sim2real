"""Save a self-contained local replay of checked reference poses; no live control."""
import argparse
import json
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--evidence',type=Path,required=True)
    p.add_argument('--svg',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    result=json.loads((args.evidence/'result.json').read_text(encoding='utf-8'))
    if not result['passed']: raise ValueError('Passing evidence required')
    figure=json.loads(args.svg.with_suffix('.json').read_text(encoding='utf-8'))
    if figure['route_version']!=result['route_version'] or Path(figure['evidence']).resolve()!=(args.evidence/'result.json').resolve():
        raise ValueError('Figure and replay evidence differ')
    data={}
    for key in ('exchange','stale_exit'):
        frames=json.loads((args.evidence/(key+'-frames.json')).read_text(encoding='utf-8'))
        data[key]={'frames':[[round(f['t'],3),*[ [round(v,6) for v in f['poses'][r]] for r in ('pinky1','pinky2')],
                              f['states']['pinky1'],f['states']['pinky2']] for f in frames],
                   'events':result[key]['events']}
    svg=args.svg.read_text(encoding='utf-8')
    svg=svg.replace('width="1480" height="1450" viewBox="0 0 1480 1450"',
                    'id="scene" viewBox="50 180 1400 950" role="img" aria-label="두 핑키의 경로 재생"')
    marks=[]
    for rid,color,fill in [('pinky1','#b4373f','#f6d8dc'),('pinky2','#344fc3','#dce4ff')]:
        marks.append(f'<g id="{rid}" stroke="{color}" stroke-width=".005" fill="{fill}">'
                     '<rect x="-.2309777993" y="-.0635" width=".155" height=".127"/>'
                     '<rect x="-.0759777993" y="-.055" width=".1180278419" height=".11"/>'
                     '<path d="M.005 -.03 L.035 0 L.005 .03" fill="none"/></g>'
                     f'<text id="{rid}-label" fill="{color}" font-size="25" font-family="Malgun Gothic,sans-serif">{rid[-1]}</text>')
    svg=svg.replace('</svg>',''.join(marks)+'</svg>')
    html='''<!doctype html>
<html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Pinky 도로 v3 · 좌하단 기본 대기</title>
<style>
body{margin:0;background:#f6f7fa;color:#17283b;font-family:"Malgun Gothic",system-ui,sans-serif}
main{max-width:1120px;margin:auto;padding:24px}h1{font-size:25px;margin:0 0 10px}p{line-height:1.6}
.controls{display:flex;gap:12px;align-items:center;flex-wrap:wrap}button,select{font:inherit;padding:8px 12px;border:1px solid #a4afbd;border-radius:5px;background:white;color:inherit}
button:focus-visible,select:focus-visible,input:focus-visible{outline:3px solid #3458bf;outline-offset:3px}
#seek{width:100%;margin:15px 0}#time{font-variant-numeric:tabular-nums}.status{display:flex;gap:24px;flex-wrap:wrap;margin:10px 0}
#state1{color:#a62c36}#state2{color:#304bbe}#scene{display:block;width:100%;height:auto;background:white}.note{font-size:14px;color:#45576b}
</style><main><h1>좌하단 기본 대기 → 적재 → 교대 하역 → 복귀 · 도로 v3</h1>
<p>차로 중심 간격 24cm. 오른쪽 앞차부터 출발하고 뒤차가 앞으로 이동합니다. 보라색은 기본 대기줄과 진출입 경로입니다.</p>
<div class="controls"><label>시나리오 <select id="scenario"><option value="exchange">전체 운행 · 기본 대기 출발/복귀</option><option value="stale_exit">이탈 정보 오래됨 → 정지</option></select></label>
<button id="play" type="button">재생</button><button id="reset" type="button">처음</button>
<label>속도 <select id="speed"><option value="1">1배</option><option value="4" selected>4배</option><option value="8">8배</option></select></label>
<output id="time" aria-live="off"></output></div>
<label for="seek">모사 시간</label><input id="seek" type="range" min="0" max="0" step="0.025" value="0">
<div class="status" aria-live="polite"><span id="state1"></span><span id="state2"></span></div>
__SVG__
<p class="note">빨강 1번·파랑 2번. 옅은 사각형은 목표 자리입니다. 기본 대기줄은 앞차 우선이며, 적재 후에는 A/B에서 인계 순서를 기다립니다. 적재·하역은 각각 3초로 모사합니다.</p>
<p class="note">바구니 포함 외곽에 각 2cm를 더한 기준 경로·시간 검사입니다. Isaac 바퀴 추종·실제 카메라·로봇팔 실행은 포함하지 않으며, 실기 정확도와 처리량을 뜻하지 않습니다.</p>
</main><script>
const data=__DATA__;
const select=document.getElementById('scenario'),seek=document.getElementById('seek'),play=document.getElementById('play');
const speed=document.getElementById('speed'),clock=document.getElementById('time');
const labels={home_wait:'좌하단 기본 대기',queue_advancing:'대기줄 앞으로 이동',to_loading:'적재 장소 이동',loading:'적재 중 · 모사',waiting_loaded:'적재 완료 · 인계 순서 대기',to_handoff:'인계 이동',unloading:'하역 중 · 모사',returning:'기본 대기구역 복귀',complete:'기본 대기 복귀 완료',fault_stop:'정보 이상 · 정지/점유 유지'};
let playing=false,last=null,t=0;
function stop(){playing=false;last=null;play.textContent='재생';}
function draw(){
 const run=data[select.value],frames=run.frames,end=frames[frames.length-1][0];
 t=Math.min(Math.max(0,t),end);seek.max=end;seek.value=t;
 let lo=0,hi=frames.length-1;while(lo<hi){const m=Math.ceil((lo+hi)/2);if(frames[m][0]<=t)lo=m;else hi=m-1;}
 const f=frames[lo];document.getElementById('scene').dataset.frame=lo;
 for(let i=0;i<2;i++){
   const rid='pinky'+(i+1),p=f[1+i],x=100+600*p[0],y=200-600*p[1];
   document.getElementById(rid).setAttribute('transform',`translate(${x} ${y}) rotate(${-p[2]*180/Math.PI}) scale(600 -600)`);
   const lab=document.getElementById(rid+'-label');lab.setAttribute('x',x+12);lab.setAttribute('y',y-18);
   let label=labels[f[3+i]];
   const waiting=run.events.find(e=>e.robot===rid&&Number.isFinite(e.start)&&e.t<=t&&t<e.start);
   if(waiting&&f[3+i]!=='fault_stop')label=waiting.event==='departure_scheduled'?'복귀 차량 합류 대기':'앞차 통과 · 합류 대기';
   document.getElementById('state'+(i+1)).textContent=(i+1)+'번: '+label;
 }
 clock.textContent=t.toFixed(2)+' / '+end.toFixed(2)+'초';
}
function animate(now){if(!playing)return;if(last!==null)t+=(now-last)/1000*Number(speed.value);last=now;draw();if(t>=Number(seek.max)){stop();return;}requestAnimationFrame(animate);}
play.addEventListener('click',()=>{if(playing){stop();return;}if(t>=Number(seek.max))t=0;playing=true;last=null;play.textContent='일시정지';requestAnimationFrame(animate);});
seek.addEventListener('input',()=>{stop();t=Number(seek.value);draw();});
document.getElementById('reset').addEventListener('click',()=>{stop();t=0;draw();});
select.addEventListener('change',()=>{stop();t=0;draw();});draw();
</script></html>'''
    html=html.replace('__SVG__',svg).replace('__DATA__',json.dumps(data,separators=(',',':'),ensure_ascii=False))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(html,encoding='utf-8')
    print(json.dumps({'html':str(args.output),'bytes':args.output.stat().st_size}))


if __name__=='__main__':main()
