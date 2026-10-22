"""Package already-verified local media and evidence with a portable gallery."""
from pathlib import Path
import argparse,hashlib,html,json,shutil,subprocess

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path,default=ROOT/'deliverables/route-v3-20261022')
    p.add_argument('--reference-run',type=Path,default=ROOT/'outputs/media-route-001')
    p.add_argument('--physics-run',type=Path,default=ROOT/'outputs/media-wheel-002')
    args=p.parse_args();out=args.output.resolve()
    if (out/'manifest.json').exists():raise ValueError('Finalized package exists; choose a new version folder')
    reference=json.loads((args.reference_run/'run.json').read_text(encoding='utf-8'))
    physics=json.loads((args.physics_run/'run.json').read_text(encoding='utf-8'))
    if reference['status']!='completed_media' or physics['status']!='passed_smoke':
        raise ValueError('Successful reference render and physical capture required')
    if not physics['checks']['capture_does_not_advance_physics'] or not physics.get('video'):
        raise ValueError('Physical video timing evidence required')
    def copy(source,target):
        dest=out/target;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,dest)
    for name in ('replay.html','route-plan.svg','route-plan.png','route-plan.json'):
        copy(ROOT/'docs/figures/route-plan-v2-20261022'/name,'replay/'+name)
    for name in ('layout.json','asset-manifest.json'):copy(ROOT/'config'/name,'config/'+name)
    for name in ('result.json','route-graph.json','routes.json','exchange-frames.json','stale_exit-frames.json','verification.json'):
        copy(ROOT/'outputs/routes-v3-001'/name,'evidence/routes/'+name)
    for name in ('run.json','trajectory.json','capture-frames.json','scene.usda'):
        copy(args.physics_run/name,'evidence/physics/'+name)
    for name in ('run.json','reference-scene.usda'):copy(args.reference_run/name,'evidence/reference-render/'+name)
    for name in ('01-route-overview.mp4','02-key-moments.mp4'):copy(args.reference_run/name,'videos/'+name)
    copy(args.physics_run/'03-wheel-physics.mp4','videos/03-wheel-physics.mp4')
    for name in ('isaac-home.png','isaac-handoff.png','isaac-stop.png'):copy(args.reference_run/name,'images/'+name)
    copy(args.physics_run/'wheel-physics.png','images/wheel-physics.png')
    copy(ROOT/'docs/MEDIA_HANDOFF.md','HANDOFF.md')
    validation=(ROOT/'docs/ROUTE_VALIDATION.md').read_text(encoding='utf-8')
    validation=validation.replace('figures/route-plan-v2-20261022/','../replay/')
    validation=validation.replace('../outputs/routes-v3-001/','routes/')
    validation=validation.replace('../outputs/replay-check-v3/result.json','replay-ui-check.json')
    validation=validation.replace('../outputs/replay-v2-before-v3/replay.html','history/replay-v2.html')
    (out/'evidence/ROUTE_VALIDATION.md').write_text(validation,encoding='utf-8')
    integration=(ROOT/'docs/INTEGRATION.md').read_text(encoding='utf-8').replace('(ROAD_HANDOFF.md)','(../HANDOFF.md)').replace('(MEDIA_HANDOFF.md)','(../HANDOFF.md)')
    (out/'evidence/INTEGRATION.md').write_text(integration,encoding='utf-8')
    copy(ROOT/'outputs/replay-v2-before-v3/replay.html','evidence/history/replay-v2.html')
    for folder in ('pinky_motion','tools','tests','config','apps'):
        for source in (ROOT/folder).rglob('*'):
            if source.is_file() and '__pycache__' not in source.parts and source.suffix in ('.py','.cjs','.json','.kit') and 'baseline' not in source.parts:
                copy(source,'source/'+str(source.relative_to(ROOT)))
    copy(ROOT/'outputs/replay-check-v3/result.json','evidence/replay-ui-check.json')
    # Full decode catches truncated/invalid frames; ffprobe records stream facts.
    ffmpeg=shutil.which('ffmpeg');probe=str(Path(ffmpeg).with_name('ffprobe.exe'))
    videos=[]
    for filename,title,kind in [
        ('01-route-overview.mp4','전체 운행 · 4배속','reference_playback'),
        ('02-key-moments.mp4','대기줄·교대·이상 정지','reference_playback'),
        ('03-wheel-physics.mp4','단일 Pinky 바퀴 구동','physics_smoke')]:
        f=out/'videos'/filename
        stream=json.loads(subprocess.check_output([probe,'-v','error','-show_streams','-show_format','-of','json',str(f)],creationflags=subprocess.CREATE_NO_WINDOW))
        subprocess.run([ffmpeg,'-hide_banner','-loglevel','error','-xerror','-i',str(f),'-f','null','-'],check=True,capture_output=True,timeout=120,creationflags=subprocess.CREATE_NO_WINDOW)
        v=next(s for s in stream['streams'] if s['codec_type']=='video')
        videos.append({'file':'videos/'+filename,'title':title,'kind':kind,'duration_s':float(stream['format']['duration']),
                       'resolution':[v['width'],v['height']],'codec':v['codec_name'],'frames':int(v['nb_frames']),'full_decode_passed':True})
    keyframes=json.loads((out/'keyframes.json').read_text(encoding='utf-8'))
    README='''# Pinky 도로 v3 · 보관 자료

2026-10-22. 주요 장면 PNG 6장, 보조 Isaac/물리 시험 PNG 4장, MP4 3개와 독립 실행 HTML 재생을 모았다.

- [자료 모음 열기](index.html)
- [관제 인계 문서와 다음 세션 요청문](HANDOFF.md)
- [기존 2D 재생](replay/replay.html)
- [전체 도면](replay/route-plan.png)
- [파일 해시·영상 검사](manifest.json)

## 영상 선택

| 영상 | 용도 | 구분 |
|---|---|---|
| [전체 운행](videos/01-route-overview.mp4) | 대기→적재→교대 하역→복귀 설명 | Isaac에서 기준 자세를 배치한 3D 재생, 4배속 |
| [주요 장면](videos/02-key-moments.mp4) | 대기줄·교대·이상 정지 발표 | 구간별 2배/1배속, 영상에 배속 표시 |
| [바퀴 구동](videos/03-wheel-physics.mp4) | 구현한 물리 시험 소개 | 단일 로봇의 실제 바퀴 목표 속도/PhysX 이동, 1배속 |

앞의 두 영상은 두 로봇의 실제 바퀴 추종이나 센서 주행 결과가 아니다. 마지막 영상도 전체 도로 운행·Nav2·정밀 도킹·실물 정확도를 검증하지 않았다. 팔과 상품 적재/하역 동작은 실행하지 않는다. 바구니 색은 로봇을 구분하기 위한 표현이다.

## 정리자료에 사용할 설명

“2m × 1.5m 공간에서 바구니를 포함한 두 이동로봇의 방향별 차로와 대기줄을 설계했다. 기준 경로의 바구니 외곽과 구간 사이 이동을 검사하고, 인계 점유·이탈 확인·교대 출발·오래된 관측 정지를 모사했다. Isaac에서 동선을 시각화하고 단일 로봇의 바퀴 구동 기본 시험을 별도로 수행했다.”

36개 자동시험과 기준 경로 검사는 `evidence/routes`에, 영상 녹화 시 물리 시간을 추가로 진행시키지 않았는지 확인한 자료는 `evidence/physics`에 있다. 실물 추종·제동 오차와 센서 정밀주차는 후속 과제다.

## 원본과 재현

이 폴더의 PNG·MP4·HTML은 복사해 사용할 수 있다. `manifest.json`의 SHA-256으로 원본 여부를 확인한다. 이후 수정본은 새 날짜/버전 폴더로 저장한다.

`source`는 제작 시점 코드 스냅샷이다. 실행 원본 저장소는 `pinky-omx-sim2real`이며 기존 `C:/isaacsim`과 인접 `shopping-robot-digital-twin`의 자산을 사용한다. USD/메시 원본은 이 묶음에 복사하지 않았다. USD 장면은 외부 자산 절대 경로를 참조하므로 동영상처럼 독립 실행되는 파일은 아니다.

원본 저장소에서 새 출력 이름을 사용한다.

```powershell
python -B tools/launch_sim.py --reference-media --output outputs/media-route-next --timeout 600
python -B tools/launch_sim.py --capture-video --output outputs/media-wheel-next --timeout 360
```

이번 세션의 종료 범위는 자료 제작과 인계 준비다. 관제 코드는 수정하지 않았다. 다음은 관제에서 명령 한 건의 요청·취소·결과 왕복을 시뮬 백엔드에 연결하는 작업이다.
'''
    (out/'README.md').write_text(README,encoding='utf-8')
    parts=['''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Pinky 도로 v3 · 자료 모음</title><style>
    *{box-sizing:border-box}body{margin:0;background:#f3f6f9;color:#172d41;font-family:'Malgun Gothic',sans-serif}main{max-width:1320px;margin:auto;padding:36px 24px 70px}h1{font-size:32px;margin:0 0 10px}p{line-height:1.8}nav{display:flex;gap:12px;flex-wrap:wrap;margin:22px 0 36px}a{color:#075f78}nav a{background:white;padding:10px 16px;border-radius:8px;text-decoration:none;border:1px solid #d5e1e8}.grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:22px}.card{background:white;border:1px solid #d9e3ea;border-radius:12px;padding:18px}.card h3{font-size:21px;margin:0 0 12px}video,img{display:block;width:100%;border-radius:6px}small{display:block;line-height:1.7;color:#52687a;margin:10px 0}.tag{display:inline-block;font-size:13px;padding:4px 8px;background:#e7edf7;border-radius:5px;margin-bottom:10px}h2{margin-top:42px}.warning{background:#edf2f8;border-left:4px solid #6d86a2;padding:16px 20px}@media(max-width:800px){.grid{grid-template-columns:1fr}h1{font-size:25px}}</style><main><h1>Pinky 도로 v3 · 이미지와 영상</h1><p>2026-10-22 보관본 · 좌하단 기본 대기 / 적재 / 교대 하역 / 복귀</p><nav><a href="#videos">영상 3개</a><a href="#images">주요 장면 6장</a><a href="#isaac">Isaac 장면</a><a href="replay/replay.html">직접 재생</a><a href="HANDOFF.md">관제 인계 문서</a><a href="README.md">사용 설명</a></nav><p class="warning">두 대의 3D 영상은 계산된 기준 경로를 Isaac에서 재생한 설명 자료입니다. 별도의 바퀴 구동 영상은 단일 로봇의 물리 시험입니다. 전체 물리 추종·실기 정밀주차 완료를 뜻하지 않습니다.</p><h2 id="videos">영상</h2><div class="grid">''']
    posters=['images/isaac-home.png','images/isaac-handoff.png','images/wheel-physics.png']
    for video,poster in zip(videos,posters):
        label='기준 경로 3D 재생' if video['kind']=='reference_playback' else '단일 로봇 PhysX 시험'
        parts.append(f'<article class="card"><span class="tag">{label}</span><h3>{html.escape(video["title"])}</h3><video controls preload="metadata" poster="{poster}"><source src="{video["file"]}" type="video/mp4"></video><small>{video["duration_s"]:.2f}초 · 1280×720 · H.264</small><a href="{video["file"]}" download>MP4 저장</a></article>')
    parts.append('</div><h2 id="images">주요 장면 · 1920×1080 PNG</h2><div class="grid">')
    for shot in keyframes:
        parts.append(f'<article class="card"><h3>{html.escape(shot["title"])}</h3><a href="{shot["file"]}"><img loading="lazy" src="{shot["file"]}" alt="{html.escape(shot["title"])}"></a><small>{html.escape(shot["description"])}</small><a href="{shot["file"]}" download>PNG 저장</a></article>')
    parts.append('</div><h2 id="isaac">Isaac 장면 · 보조 PNG</h2><div class="grid">')
    for name,title in [('isaac-home','3D 기본 대기'),('isaac-handoff','3D 교대 인계'),('isaac-stop','3D 이상 정지'),('wheel-physics','바퀴 구동 시험')]:
        parts.append(f'<article class="card"><h3>{title}</h3><a href="images/{name}.png"><img loading="lazy" src="images/{name}.png" alt="{title}"></a></article>')
    parts.append('</div><p><a href="replay/route-plan.png">전체 도면</a> · <a href="manifest.json">파일 해시와 검증 정보</a></p></main></html>')
    (out/'index.html').write_text(''.join(parts),encoding='utf-8')
    manifest={'package':'pinky-route-v3-20261022','route_version':3,'created_date':'2026-10-22',
        'reference_media_is_physics_tracking':False,'physical_video_is_full_route':False,
        'videos':videos,'keyframes':keyframes,'source_asset_hashes':'config/asset-manifest.json',
        'checks':{'physics_capture_passed':True,'all_videos_fully_decoded':True},'files':[]}
    for f in sorted(out.rglob('*')):
        if f.is_file():manifest['files'].append({'path':f.relative_to(out).as_posix(),'bytes':f.stat().st_size,'sha256':hashlib.sha256(f.read_bytes()).hexdigest()})
    (out/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'package':str(out),'files':len(manifest['files']),'videos':videos},ensure_ascii=False))


if __name__=='__main__':main()
