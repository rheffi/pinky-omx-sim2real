// Verify only this task's local gallery and its three MP4 files.
const {chromium}=require('playwright');
const fs=require('node:fs');
const path=require('node:path');
const {pathToFileURL}=require('node:url');
const assert=require('node:assert/strict');
(async()=>{
  const root=path.resolve(__dirname,'..');
  const packageRoot=path.join(root,'deliverables/route-v3-20261022');
  const output=path.join(root,'outputs/media-gallery-check');fs.mkdirSync(output,{recursive:true});
  const browser=await chromium.launch({executablePath:'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe',headless:true});
  try{
    const page=await browser.newPage({viewport:{width:1400,height:1000}});const errors=[];
    page.on('pageerror',e=>errors.push(e.message));
    await page.goto(pathToFileURL(path.join(packageRoot,'index.html')).href);
    assert.equal(await page.locator('video').count(),3);
    assert.equal(await page.locator('img').count(),10);
    const videos=[];
    for(let i=0;i<3;i++){
      const v=page.locator('video').nth(i);
      await v.scrollIntoViewIfNeeded();
      const info=await v.evaluate(async el=>{
        if(el.readyState<1)await new Promise((resolve,reject)=>{el.addEventListener('loadedmetadata',resolve,{once:true});el.addEventListener('error',()=>reject(new Error('Video metadata error')),{once:true});});
        el.muted=true;await el.play();return {duration:el.duration,width:el.videoWidth,height:el.videoHeight};
      });
      await page.waitForFunction(i=>document.querySelectorAll('video')[i].currentTime>.1,i);
      await v.evaluate(async el=>{el.pause();const done=new Promise(resolve=>el.addEventListener('seeked',resolve,{once:true}));el.currentTime=el.duration/2;await done;});
      assert(info.duration>1);assert.equal(info.width,1280);assert.equal(info.height,720);
      videos.push({...info,playback_and_seek:true});
    }
    await page.locator('#images').scrollIntoViewIfNeeded();
    await page.screenshot({path:path.join(output,'gallery.png'),fullPage:true});
    assert.deepEqual(errors,[]);
    const result={passed:true,videos,images:10,page_errors:errors};
    fs.writeFileSync(path.join(output,'result.json'),JSON.stringify(result,null,2));console.log(JSON.stringify(result));
  }finally{await browser.close();}
})().catch(e=>{console.error(e);process.exitCode=1});
