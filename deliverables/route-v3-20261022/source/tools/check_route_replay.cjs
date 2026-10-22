// Local artifact UI check. Requires Playwright and an installed Edge; no server.
const { chromium } = require('playwright');
const fs = require('node:fs');
const path = require('node:path');
const { pathToFileURL } = require('node:url');
const assert = require('node:assert/strict');
(async () => {
  const root = path.resolve(__dirname, '..');
  const out = path.join(root, 'outputs/replay-check-v3');
  const input = path.resolve(root, process.argv[2] || 'docs/figures/route-plan-v2-20261022/replay.html');
  const evidence = JSON.parse(fs.readFileSync(path.join(root, 'outputs/routes-v3-001/result.json'), 'utf8'));
  fs.mkdirSync(out, { recursive: true });
  const browser = await chromium.launch({
    executablePath: 'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe',
    headless: true,
  });
  try {
    const page = await browser.newPage({ viewport: { width: 1100, height: 1100 } });
    const errors = [];
    page.on('pageerror', e => errors.push(e.message));
    await page.goto(pathToFileURL(input).href);
    const setTime = async t => page.locator('#seek').evaluate((el, value) => {
      el.value = value; el.dispatchEvent(new Event('input', { bubbles: true }));
    }, String(t));
    const initial = await page.locator('#pinky1').getAttribute('transform');
    assert.match(await page.locator('h1').textContent(), /도로 v3/);
    assert.match(await page.locator('#state1').textContent(), /기본 대기/);
    assert.match(await page.locator('#state2').textContent(), /기본 대기/);
    await page.screenshot({ path: path.join(out, 'home-start.png'), fullPage: true });
    await setTime(4);
    assert.match(await page.locator('#state1').textContent(), /적재 장소 이동/);
    assert.match(await page.locator('#state2').textContent(), /대기줄 앞으로 이동/);
    assert.notEqual(await page.locator('#pinky1').getAttribute('transform'), initial);
    const departure = evidence.exchange.events.find(e => e.robot === 'pinky2' && e.event === 'departure_scheduled');
    await setTime((departure.t + departure.start) / 2);
    assert.match(await page.locator('#state2').textContent(), /합류 대기/);
    const movingTime = departure.start + 1;
    await setTime(movingTime);
    assert.match(await page.locator('#state1').textContent(), /복귀/);
    assert.match(await page.locator('#state2').textContent(), /인계 이동/);
    await page.screenshot({ path: path.join(out, 'normal-moving.png'), fullPage: true });
    await page.locator('#play').click();
    await page.waitForFunction(t => Number(document.getElementById('seek').value) > t + .1, movingTime);
    await page.locator('#play').click();
    await setTime(evidence.exchange.elapsed_sim_seconds);
    assert.match(await page.locator('#state1').textContent(), /기본 대기 복귀 완료/);
    assert.match(await page.locator('#state2').textContent(), /기본 대기 복귀 완료/);
    assert.equal(await page.locator('#pinky1').getAttribute('transform'), initial);
    await page.screenshot({ path: path.join(out, 'home-return.png'), fullPage: true });
    await page.locator('#scenario').selectOption('stale_exit');
    await setTime(Number(await page.locator('#seek').getAttribute('max')));
    assert.match(await page.locator('#state1').textContent(), /점유 유지/);
    assert.match(await page.locator('#state2').textContent(), /점유 유지/);
    assert.deepEqual(errors, []);
    const summary = { passed: true, slider_updates_pose: true, merge_wait_visible: true,
      concurrent_reference_motion_visible: true, playback_advances: true,
      home_queue_departure_visible: true, returned_to_home_visible: true,
      stale_exit_stops_both: true, page_errors: errors, input };
    fs.writeFileSync(path.join(out, 'result.json'), JSON.stringify(summary, null, 2));
    console.log(JSON.stringify(summary));
  } finally { await browser.close(); }
})().catch(e => { console.error(e); process.exitCode = 1; });
