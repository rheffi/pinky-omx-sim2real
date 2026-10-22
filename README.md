# Pinky OMX Sim-to-Real

기존 쇼핑 관제에 통합할 Pinky Pro 2대의 주행·교통 제어·후진 정밀주차 개발 저장소다. **2026-10-22 초기 구현에 착수했다.**

- [전체 계획](PLAN.md)
- [v4 짧은 동선 검토 후보 · 관제 미반영](docs/COMPACT_ROUTE_REVIEW.md) · [새 2D 재생](docs/figures/route-plan-v4-review-20261022/replay.html)
- [현재 구현과 실행 결과](docs/IMPLEMENTATION.md)
- [도로 평면도·관제로 넘길 범위](docs/ROAD_HANDOFF.md)
- [도로 v3 검증·좌하단 기본 대기와 전체 운행 재생](docs/ROUTE_VALIDATION.md)
- [도로 v3 이미지·Isaac 영상 자료 모음](deliverables/route-v3-20261022/index.html)
- [자료 보관 완료 기록](docs/MEDIA.md) · [관제 통합 인계](docs/MEDIA_HANDOFF.md)
- [바구니 포함 회전 9조건 비교](docs/BASKET_ROTATION.md)
- [확보한 실기 기준 자료](docs/BASELINE.md)
- [관제·ROS 연결 경계](docs/INTEGRATION.md)
- [현장 배치 설정](config/layout.json)
- [기존 배치 이미지](map-layout-2026-10-22.png): 이후 확인한 카메라 벽 거리 12cm 등은 설정 JSON을 우선한다.

## 현재 구성

| 경로 | 역할 |
|---|---|
| `tools/sim_smoke.py` | 기존 Isaac Sim에서 단일 Pinky와 후면 바구니의 물리 주행·정지 시험, 장면/이미지/궤적 저장 |
| `tools/basket_sweep.py` | 바구니 질량·접촉·마찰·높이를 분리한 9조건 회전 비교, 접촉력 기록 |
| `tools/launch_sim.py` | 로컬 캐시·localhost ROS 도메인·시간 제한을 설정한 실행기 |
| `pinky_motion/ros_loopback.py` | 신규 시뮬 전용 명령, 정답 자세·TF·관절·시각 송수신 시험 |
| `pinky_motion/geometry.py`, `routes.py` | 바구니 포함 외곽, 목표 접합부 좌표, 방향별 도로 후보와 기하 검사 |
| `pinky_motion/route_paths.py`, `route_validation.py` | 작업대 6개 이동 경로와 보간 구간의 보수적 외곽 검사 |
| `pinky_motion/fleet_reference.py`, `tools/check_routes.py` | 좌하단 기본 대기·적재·교대 인계·복귀의 시간 경로 모사·예약·이상 정지 검사 |
| `pinky_motion/docking.py` | 영상 소실·기준 변경·오래된 프레임에서 정차 허가를 막는 관측 판정 모듈 |
| `pinky_motion/traffic.py` | 원자적 구역 예약·FIFO 대기·관측 후 해제·명령 수신 watchdog 프로토타입 |
| `tests/test_motion.py` | 교통 충돌 요청·통신 불명·중복·가짜 이탈·바구니 외곽·정지 로직 시험 |
| `tools/offline_check.py` | 도로 외곽 검사와 두 로봇 인계 요청의 합성 사건 기록 |

정밀 도킹·Nav2 Route Server 연결·기존 관제 통합·두 로봇의 물리 충돌 시험은 아직 완료하지 않았다. 시뮬 정답 자세는 실물 센서 기반 위치 추정 결과가 아니다.

## 실행

현재 Windows 환경에서 설치 없이 사용하는 명령이다. Python 3.12와 기존 `C:/isaacsim`이 필요하다. 작업 위치는 이 저장소 루트다. 실행 폴더는 새 이름을 사용한다.

```powershell
python -B -m unittest discover -s tests -v
python -B tools/offline_check.py --output outputs/offline-next
python -B tools/check_routes.py --output outputs/routes-next
python -B tools/launch_sim.py --output outputs/smoke-next --ros --timeout 360
python -B tools/launch_sim.py --basket-sweep --output outputs/basket-sweep-next --timeout 360
```

실행기는 로봇 도메인 1/2 대신 simulation 도메인 83과 localhost를 사용한다. `--ros` 없이 물리 시험만 할 수도 있다. Ctrl+C로 직접 끝낸 실행은 성공으로 인정하지 않는다. 프로세스 종료 코드가 0이어도 `run.json`의 검증 결과가 실패면 실행기는 실패를 반환한다.

기본 물리 설정은 실측 보정 전 가정이다. `--drive-damping 2.0`처럼 변경한 실험값은 결과의 config에 저장되며 기본 설정 파일을 덮어쓰지 않는다.

`--basket-sweep`은 별도 비교용 바닥에서 이전 접촉 모델들을 시험하며 ROS 옵션과 함께 사용하지 않는다. 완료 상태 `completed_experiment`는 비교 자료 수집 성공을 뜻한다. 후속 사용자 결정에 따라 현재 기본 모델은 바구니-바닥 접촉을 제외하고 벽·물건·다른 로봇 충돌을 유지한다. 5mm 간격은 표현용 가정이다. 이 기본 모델의 단일 전후진·회전·정지 시험은 통과했으며 정밀주차·두 대 주행 완료를 뜻하지 않는다.

## 기존 프로젝트와의 관계

- `../shopping-robot-digital-twin`: 변환된 공식 Pinky/OMX USD를 참조한다. 원본은 변경하지 않는다. 실행 USD는 현재 개발 PC의 원본 자산 경로에 의존하므로 배포 시 자산 패키징·라이선스 검토가 필요하다.
- `../kernel-robot-repo-3` / `rheffi/cartrina-control`: 최종 Vue/FastAPI 관제. 주문·재고·고객 배정·YOLO/ACT를 재구현하지 않는다. 이번에 수정하지 않았다.
- 이 저장소: 동적 주행·도로/예약·도킹 비교·관제 이동 어댑터를 개발하고 검증 결과를 옮긴다.

개인 비공개 원격은 `https://github.com/rheffi/pinky-omx-sim2real`이다. 코드·설정·문서·선별한 인계 자료를 Git으로 백업하며 `outputs/`와 `.runtime/`은 제외한다. Isaac 원본 USD·변환 자산과 설치 환경은 별도 경로에 있으므로 이 저장소만 clone해서 모든 물리 시뮬을 바로 실행할 수 있다고 보지 않는다. 관제용으로 가져온 기준 경로는 팀 저장소에서 별도로 버전을 관리한다.

## 다음 순서

**2026-10-22 추가 검토:** 팀원 영상 피드백에 따라 큰 순환 우회를 줄인 v4 후보를 별도로 작성했다. A→B→인계 순서, 한 대씩 이동, 좌하단 대기줄, 짧은 작업대 연결을 검토한다. 기존 v3는 보존하고, 사용자가 새 재생을 확인하기 전까지 이 후보를 관제 저장소·대화 세션에 반영하지 않는다. [후보 검토 기록](docs/COMPACT_ROUTE_REVIEW.md)을 먼저 확인한다.

도로 v3는 24cm 차로 간격과 좌하단 기본 대기줄을 사용한다. 작업대 간 6개·대기줄 관련 10개 경로와 두 대의 기본 대기→적재→교대 하역→복귀 모사를 검사했고 자동시험 36개가 통과했다. 이미지 10장·영상 3개·원본 재생·검증 근거를 별도 보관하여 이번 세션 범위를 마무리했다.

다음은 기존 관제 저장소에서 시뮬 이동 명령 한 건의 요청·취소·결과 왕복을 연결한다. 관제는 같은 도로 버전과 교통 규칙을 사용한다. 전체 Isaac 바퀴 경로 추종·정지·센서 도킹·실물 전이는 후속 검증으로 남는다. [관제 인계 문서](docs/MEDIA_HANDOFF.md)의 범위를 기준으로 계속한다.
