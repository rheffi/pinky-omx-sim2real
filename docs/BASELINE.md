# 실기 기준 자료 — 2026-10-22

현장 자료 수집은 사용자의 요청으로 종료했다. 아래 확보된 자료로 초기 구현을 진행한다. 로봇에 접속하거나 설정을 바꾼 결과가 아니다.

## 보관 원본

Downloads의 다음 압축파일을 앞선 읽기 전용 분석에서 확인했다. 압축 내부 해시 목록은 각각 86/34/154개 항목이 일치했다. 원본 파일은 변경하지 않았다.

- `pinky_diagnostics_20261022_110202_baseline.tar.gz`: Pinky1 저장 설정·과거 로그. 수집 당시 live 주행 없음.
- `Pinky2_주행전_사전조사_20261022.tar.gz`: Pinky2 저장 설정. 수집 당시 live 주행 없음.
- `Pinky2_A후진주차2회_전진복귀1회_20261022_123214.tar.gz`: Pinky2 실제 후진 2회·전진 복귀 1회, MCAP·목표/결과·런타임 설정.

마지막 원본 경로: `C:/Users/cch97/Downloads/Pinky2_A후진주차2회_전진복귀1회_20261022_123214.tar.gz`.

## 확인한 설정

| 항목 | 확인된 값 |
|---|---|
| ROS / Nav2 | Jazzy / 1.3.12 |
| ROS_DOMAIN_ID | Pinky1=1, Pinky2=2 |
| 지도 | `/home/pinky/pinky_pro/pinky_map_new.yaml`, 두 로봇의 YAML/PGM 동일 |
| 지도 해상도·크기·원점 | 0.01m, 164×212, [-0.236,-0.163,0] |
| 경로 추종 | Regulated Pure Pursuit, desired_linear_vel=0.15m/s, 20Hz |
| 도착 판정 | SimpleGoalChecker, xy=0.08m, yaw=0.25rad, stateful=true |
| 저장된 후진 설정 | allow_reversing=false, use_rotate_to_heading=true |
| 후속 시험 런타임 | allow_reversing=true, use_rotate_to_heading=false; 저장 YAML은 그대로 |
| 충돌 외곽 | ±0.06m 사각형, padding=0.01m; 후면 바구니 미포함 |
| inflation 비용 계수 | Pinky1=10, Pinky2=3 (RPP 대응 항목과 local/global) |
| 엔코더 기구 상수 | 반지름 0.027m, 간격 0.0961m |
| 기본 TF | AMCL map→odom, 구동부 odom→base_footprint |
| IMU | 후속 실행에서 사용 확인 안 됨 |

두 로봇에서 일치한 지도 SHA256:

- YAML: `3f1a1f1c2f3dc28167194c7c43a953b2bb949d3a72c0f4f668521a73d01c13dd`
- PGM: `3693a6f7275226cd8c72a764f98a2fc13fcd02ee1eb8d4731a790548a5930aa5`

## 실제 주행 결과와 해석

| 시험 | Nav2 결과 | 실행 시간 | 최종 지도 추정 위치와 목표 거리 | 최종 추정 방향 차이 |
|---|---|---:|---:|---:|
| A 후진 1 | status 4 / error 0 | 4.81초 | 10.58cm | +14.08° |
| 전진 복귀 | status 4 / error 0 | 4.52초 | 6.62cm | +17.09° |
| A 후진 2 | status 4 / error 0 | 5.14초 | 11.79cm | -6.59° |

MCAP의 선택한 TF·odom·속도 메시지를 별도 파싱해 이동 거리와 최종 추정 오차가 보고서와 일치함을 확인했다. 위 표는 외부 측정 기준의 실제 바구니 오차가 아니다. 접합부 벽 거리·횡방향·각도 실측 항목은 null이었다.

- 사용자는 과거 20~30cm 표현이 과장이었다고 정정했다. 진단 근거에서 제외한다.
- 초기 취소된 약 5mm 시험은 수집기의 오판으로 취소한 기록이며 성공/실패 주행 통계에서 제외한다.
- 실제 A 후진 중 음수 선속도를 확인했다. 별도 180도 회전 단계가 있었던 시험은 아니다.
- SimpleGoalChecker의 stateful XY 판정과 목표를 odom으로 변환한 시점, 이후 map→odom 갱신을 구분해야 한다. 최종 지도 오차가 8cm보다 크다는 사실만으로 도착 판정 버그라고 단정할 수 없다.
- 보고서의 도착 시점 근사 재구성은 XY 약 7.59~7.99cm와 yaw 허용 범위 안이었다. 컨트롤러 내부 TF 캐시의 정확한 판정 순간을 기록한 값은 아니다.
- 외부 기준 위치가 없어 AMCL·엔코더·미끄러짐의 오차 기여도를 확정할 수 없다.
- 당시 전압 약 7.02~7.05V, 잔량 27~32%. 한 차례 제어 루프 지연 경고는 지속 과부하의 증거가 아니다.

## 다음 실물 비교 시 필요한 측정

지금 구현의 선행 조건은 아니다. 전이 시 A 후진 1회부터 접합부 벽 거리·중앙선 편차·방향을 외부 기준으로 측정하고 동일 시각의 추정 자세와 비교한다. 원인 조사와 팔 허용 범위 확정은 그 결과를 기준으로 한다.

소스 근거:

- [Nav2 1.3.12 goal checker](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_controller/plugins/simple_goal_checker.cpp)
- [Nav2 1.3.12 controller](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_controller/src/controller_server.cpp)
- [동일 버전 Route Server 패키지](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_route/package.xml): 소스 존재와 실제 장비 설치 여부는 별개다.
