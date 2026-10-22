# 관제와 ROS 연결 경계

기존 `kernel-robot-repo-3`를 수정하지 않고 준비한 연결 계약이다. 아직 관제 통합 구현이 아니다.

도로 배치·회전·진출입·대기 위치는 이 저장소에서 검증하고, 최종 관제는 같은 도로 설정을 읽어 단일 권한으로 예약과 주문 실행을 관리한다. 초기 명령 왕복과 최종 인계 기준은 [도로 설계와 관제 경계](ROAD_HANDOFF.md)에 정리했다.

## 현행 실행 계약

기존 `robot_gateway/hardware/worker.py`의 `mobile_factory`는 `execute(command, cancelled)` 객체를 연결한다. 내부 명령 `NAVIGATE`, `LEAVE`, `STOP`과 ROS 메시지/액션은 구분한다. `interfaces.py`의 navigate/leave/stop Protocol과 실제 실행 진입점 차이는 최종 통합 시 맞춰야 한다.

시뮬 이동 완료 → 정밀 정차 확인 → 팔 작업 허가 → 팔 이탈 → 출발 → 바구니 완전 이탈 → 점유 해제를 각각 다른 사건으로 처리한다. Nav2 결과나 프로세스 종료만으로 STOPPED/EXITED를 만들어내지 않는다.

기존 hardware worker와 EvidenceFiles는 hardware 모드 및 vision/feedback/operator 출처를 요구한다. 현재 관제 API의 모드는 mock/hardware이며 일부 mock 장비 시험은 simulation 출처를 사용한다. 첫 연결은 mock 하위의 시뮬 백엔드와 명시적인 실행 출처를 권장한다. 필드·검증 계약은 관제 코드에서 정리하며 모의 결과를 실물 EvidenceFiles로 전달하지 않는다. [최신 코드 확인과 인계 범위](MEDIA_HANDOFF.md)를 따른다.

## 이번에 만든 simulation 전용 인터페이스

`tools/launch_sim.py --ros`는 ROS_DOMAIN_ID=83, localhost 통신을 설정한다. Pinky 실기 도메인 1/2와 구분한다. 현재 동일 프로세스 안의 서로 다른 ROS 노드 사이 RMW 송수신 시험이다.

| 상대 토픽 (namespace `/pinky_sim/pinky_1`) | 형식 | 의미 |
|---|---|---|
| `cmd_vel` | geometry_msgs/Twist | 새 simulation 전용 명령. Pinky 드라이버 호환 확인 아님 |
| `ground_truth/odom` | nav_msgs/Odometry | PhysX 자세. 센서 기반 오도메트리 아님; 현재 twist는 제공하지 않음 |
| `ground_truth/tf` | tf2_msgs/TFMessage | sim_world→pinky_1/base_link 정답 변환. 현재 planar yaw만 전송 |
| `clock` | rosgraph_msgs/Clock | 명시적 테스트 step의 시뮬 시간, 전체 앱 초기화/렌더 프레임 시간 아님 |
| `joint_states` | sensor_msgs/JointState | USD 관절 위치. 실기 엔코더 검증 아님 |

현재 전역 `/clock`, `/tf`나 Nav2 map/odom 체인을 제공하는 완성 bridge가 아니다. Nav2 연결 단계에서 시뮬 전체 시간과 메시지 타임스탬프, 바퀴 기반 odom, 실제 센서 모델을 연결하고 ground truth는 평가 채널에 남겨야 한다.

VelocityWatchdog는 수신 단조 시계 기준 0.3초 뒤 0 명령을 반환한다. 값과 속도 한도는 초기 개발 설정이다. 호스트 프로세스가 정지한 상황을 실제 모터 watchdog이 처리한다는 의미가 아니다.

## 교통 권한

`Reservations`는 단일 프로세스의 원자적 자원 묶음과 충돌하는 요청의 FIFO를 시험하는 프로토타입이다. 최종 통합에서는 기존 관제의 작업대 점유와 하나의 권한 아래 연결한다.

- 진입 전에 작업대·교차부·출구 공간을 함께 예약한다.
- 하역 완료는 이탈이 아니다. 바구니까지 구역을 비웠다는 신선한 관측을 요구한다.
- 통신 단절·타임아웃만으로 점유를 해제하지 않는다.
- 같은 명령 ID를 재사용해 동작을 중복 실행하지 않는다.
- 현재는 boolean 관측 입력을 사용하는 단위시험이다. 실제 시간·명령 ID·로봇 ID·관측 출처 검증은 어댑터 경계에 추가해야 한다.
- 영속 저장·관제 재시작 복원·분산 합의·실제 로봇 정지/경로 추종은 아직 없다.

초기 연결 목표는 관제 명령 1개와 시뮬 결과/취소의 왕복이다. 주문·재고 엔진을 이 저장소에 복제하지 않는다. 관제의 Unix 신선도와 시뮬 정지/리셋을 연결할 때 실행 ID, 단조 수신 시각, 시뮬 시각을 별도로 보관한다.

## 도킹 판정 모듈

`DockingGate`는 보정된 작업대 기준의 바구니 자세·속도·측정 시각·카메라 기준 ID를 받는다. `reference_valid`를 계산할 실제 영상 기준점 검출은 아직 없다. 값이 없거나 신선하지 않으면 정차를 허가하지 않는다. 시험의 1cm/각도 기준은 합성 입력용이며 실제 팔 허용오차로 설정하지 않는다. 이 모듈은 검출기·영상 보정·모터 정지 회로를 대신하지 않는다.
