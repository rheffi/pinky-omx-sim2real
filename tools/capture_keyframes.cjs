// Render six presentation-ready stills from our local, verified replay.
const { chromium } = require('playwright');
const fs = require('node:fs');
const path = require('node:path');
const { pathToFileURL } = require('node:url');
(async () => {
  const root=path.resolve(__dirname,'..');
  const output=path.resolve(root,process.argv[2]||'deliverables/route-v3-20261022');
  fs.mkdirSync(path.join(output,'images'),{recursive:true});
  const evidence=JSON.parse(fs.readFileSync(path.join(root,'outputs/routes-v3-001/result.json'),'utf8'));
  const exchange=evidence.exchange.events;
  const switchTime=exchange.find(e=>e.event==='departure_scheduled'&&e.robot==='pinky2').start+1;
  const stopTime=evidence.stale_exit.events.find(e=>e.event==='exit_observation').t+2;
  const shots=[
    ['01-layout-home','전체 배치와 기본 대기','exchange',0,'200 × 150cm 공간에 두 차로와 세 작업대를 배치했습니다. 기본 대기줄은 좌하단에 있습니다.',['차로 중심 간격 24cm','바구니 포함 외곽 검사','선두 출발 · 후열 전진']],
    ['02-queue-departure','앞차 출발과 대기줄 이동','exchange',4,'1번이 적재 A로 출발하면 2번이 앞으로 이동합니다. 두 차량의 전체 시간 경로를 함께 검사합니다.',['빨강 1번: 적재 A로 이동','파랑 2번: 선두 자리로 이동','본선 합류 전 필요한 시간 대기']],
    ['03-handoff-wait','한 대 하역 · 다른 한 대 대기','exchange',94,'1번이 인계 구역을 사용하는 동안 2번은 적재 B 내부에서 기다립니다. 하역 완료와 구역 이탈은 구분합니다.',['인계 구역은 한 대씩 사용','대기 차량은 출구를 막지 않음','적재·하역은 각 3초 사건으로 모사']],
    ['04-handoff-exchange','이탈 확인 후 교대 진입','exchange',switchTime,'1번 바구니까지 구역을 비웠다는 합성 관측 후 예약을 해제합니다. 복귀 경로를 고려해 2번의 출발 시간을 조정합니다.',['1번: 기본 대기줄로 복귀','2번: 인계 구역으로 이동','이탈 후에도 추가 합류 대기 적용']],
    ['05-home-return','운행 종료와 기본 대기 복귀','exchange',evidence.exchange.elapsed_sim_seconds,'두 대가 하역을 마친 뒤 기본 대기줄로 복귀했습니다. 이 예제는 1번이 선두, 2번이 후열로 돌아오는 순서입니다.',['인계 예약 모두 해제','로봇별 고정 독립 주차석은 아님','계산된 소요시간은 실물 처리량이 아님']],
    ['06-stale-stop','오래된 이탈 정보에서 정지','stale_exit',stopTime,'이탈 확인 정보가 오래되면 두 대를 정지시키고 기존 점유를 유지합니다. 타임아웃만으로 다음 차량을 들여보내지 않습니다.',['두 로봇 정지 자세 유지','1번 인계 점유 보존','2번 진입 보류']]
  ];
  const browser=await chromium.launch({executablePath:'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe',headless:true});
  try {
    const source=await browser.newPage({viewport:{width:1280,height:1080}});
    await source.goto(pathToFileURL(path.join(root,'docs/figures/route-plan-v2-20261022/replay.html')).href);
    const page=await browser.newPage({viewport:{width:1920,height:1080},deviceScaleFactor:1});
    const metadata=[];
    for(const [name,title,scenario,t,description,points] of shots){
      await source.locator('#scenario').selectOption(scenario);
      await source.locator('#seek').evaluate((el,t)=>{el.value=t;el.dispatchEvent(new Event('input',{bubbles:true}));},t);
      const svg=await source.locator('#scene').evaluate(el=>el.outerHTML);
      await page.setContent(`<!doctype html><html lang="ko"><meta charset="utf-8"><style>
      *{box-sizing:border-box}body{margin:0;background:#f4f7fa;color:#162c40;font-family:'Malgun Gothic',sans-serif;padding:32px 36px;width:1920px;height:1080px}
      header{display:flex;align-items:center;justify-content:space-between;height:64px;margin-bottom:20px}h1{font-size:38px;margin:0}.badge{font-size:22px;color:#476579}
      main{display:grid;grid-template-columns:1250px 1fr;gap:32px}.map{background:white;border:1px solid #d8e1e8;border-radius:14px;overflow:hidden}.map svg{display:block;width:100%;height:auto}
      aside{background:white;border-radius:14px;padding:36px 32px;border-top:7px solid #087d88}aside h2{font-size:29px;margin:0 0 26px}aside p{font-size:27px;line-height:1.75;margin:0 0 32px}ul{padding-left:28px;font-size:25px;line-height:1.7}li{margin:16px 0}.stamp{margin-top:48px;font-size:23px;color:#526b7d}
      footer{font-size:23px;color:#526b7d;margin-top:28px;display:flex;justify-content:space-between}
      </style><header><h1>${title}</h1><div class="badge">Pinky × OMX · 도로 v3</div></header><main><section class="map">${svg}</section><aside><h2>장면 설명</h2><p>${description}</p><ul>${points.map(x=>`<li>${x}</li>`).join('')}</ul><div class="stamp">시나리오: ${scenario==='exchange'?'정상 운행':'이탈 정보 이상'}<br>모사 시각: ${t.toFixed(2)}초</div></aside></main><footer><span>계산된 기준 경로 · 각 로봇 외곽 2cm 확장 검사 · 물리 추종·실기 검증 아님</span><span>2026-10-22</span></footer></html>`);
      await page.evaluate(()=>document.fonts.ready);
      await page.screenshot({path:path.join(output,'images',name+'.png')});
      metadata.push({file:'images/'+name+'.png',title,scenario,source_time_s:t,description,points,kind:'reference_playback_keyframe',resolution:[1920,1080]});
    }
    fs.writeFileSync(path.join(output,'keyframes.json'),JSON.stringify(metadata,null,2));
    console.log(JSON.stringify({images:metadata.length,resolution:[1920,1080],output}));
  }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1});
