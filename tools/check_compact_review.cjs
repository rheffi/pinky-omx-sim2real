// Own local artifact only. No browser profile, external service or live control.
const { chromium } = require('playwright');
const fs = require('node:fs');
const path = require('node:path');
const { pathToFileURL } = require('node:url');
const assert = require('node:assert/strict');
(async()=>{
  const root=path.resolve(__dirname,'..');
  const folder=path.join(root,'docs/figures/route-plan-v4-review-20261022');
  const out=path.join(folder,'images');fs.mkdirSync(out,{recursive:true});
  const browser=await chromium.launch({executablePath:'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe',headless:true});
  try{
    const page=await browser.newPage({viewport:{width:1440,height:1200},deviceScaleFactor:1});
    const errors=[];page.on('pageerror',e=>errors.push(e.message));
    await page.goto(pathToFileURL(path.join(folder,'replay.html')).href);
    const setTime=async t=>page.locator('#seek').evaluate((el,t)=>{el.value=t;el.dispatchEvent(new Event('input',{bubbles:true}));},String(t));
    const capture=async file=>page.screenshot({path:path.join(out,file),fullPage:true});
    assert.match(await page.locator('h1').textContent(),/옆 작업대로/);
    const initial=await page.locator('#pinky1').getAttribute('transform');
    await capture('01-overview.png');
    await page.locator('#scene').screenshot({path:path.join(out,'route-map.png')});
    await page.locator('[data-jump="transfer"]').click();
    assert.match(await page.locator('#state1').textContent(),/적재 A → 적재 B/);
    assert.notEqual(await page.locator('#pinky1').getAttribute('transform'),initial);
    await capture('02-short-transfer.png');
    const before=Number(await page.locator('#seek').inputValue());
    await page.locator('#play').click();
    await page.waitForFunction(t=>Number(document.getElementById('seek').value)>t+.15,before);
    await page.locator('#play').click();
    await page.locator('[data-jump="wait"]').click();
    assert.match(await page.locator('#state2').textContent(),/인계 순서 대기/);
    assert.match(await page.locator('#state1').textContent(),/하역 중/);
    await capture('03-handoff-wait.png');
    await page.locator('[data-jump="return"]').click();
    assert.match(await page.locator('#state1').textContent(),/인계 → 대기 선두/);
    assert.match(await page.locator('#state2').textContent(),/인계 순서 대기/);
    await capture('04-return.png');
    await page.locator('[data-jump="end"]').click();
    for(const id of ['state1','state2'])assert.match(await page.locator('#'+id).textContent(),/복귀 완료/);
    assert.equal(await page.locator('#pinky1').getAttribute('transform'),initial);
    await page.locator('#scenario').selectOption('stale');
    await page.locator('[data-jump="end"]').click();
    for(const id of ['state1','state2'])assert.match(await page.locator('#'+id).textContent(),/관측 소실/);
    assert.match(await page.locator('#permit').textContent(),/허가 유지/);
    await capture('05-stale-stop.png');
    await page.locator('#scenario').selectOption('skip');
    assert.equal(await page.locator('#task1').textContent(),'방문: 적재 A → 인계');
    assert.equal(await page.locator('#task2').textContent(),'방문: 적재 B → 인계');
    await page.locator('[data-jump="end"]').click();
    assert.match(await page.locator('#state2').textContent(),/복귀 완료/);
    await page.locator('#scenario').selectOption('baseline');
    await page.locator('[data-jump="transfer"]').click();
    assert.match(await page.locator('#explanation').textContent(),/기존 v3/);
    await capture('06-v3-comparison.png');
    await page.setViewportSize({width:700,height:1000});
    const overflow=await page.evaluate(()=>document.documentElement.scrollWidth>innerWidth+1);
    assert.equal(overflow,false);
    assert.deepEqual(errors,[]);
    const result={passed:true,scenarios_checked:['normal','stale','skip','baseline'],
      controls:['play_pause','seek','jump','scenario','responsive_layout'],
      images:fs.readdirSync(out).filter(f=>f.endsWith('.png')),page_errors:errors};
    fs.writeFileSync(path.join(folder,'evidence/ui-check.json'),JSON.stringify(result,null,2));
    console.log(JSON.stringify(result));
  }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1;});
