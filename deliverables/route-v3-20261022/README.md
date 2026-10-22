# Pinky 도로 v3 · 보관 자료

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
