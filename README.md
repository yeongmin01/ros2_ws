# VCU–Isaac Sim 연동 ROS 2 드라이버 스택

VCU(제어기)와 NVIDIA Isaac Sim 사이의 하드웨어 에뮬레이션 및 원격 제어를 위한 ROS 2 패키지 모음입니다.  
실제 CAN 버스 통신과 시뮬레이터 사이의 중간 계층 역할을 하며, 8륜 AGV를 대상으로 개발되었습니다.

---

## 전체 아키텍처

```
[RemoteController]
        │  /j1939_tx (키보드 → 속도·조향 명령)
        ▼
[VCU] ──CAN bus──► [can_driver (can0/can1)]
                            │
              ┌─────────────┴─────────────┐
              ▼                           ▼
       [j1939_parser]             [canopen_parser]
              │ /j1939_rx                 │ /canopen_rx
              ▼                           ▼
       [motor_driver]             [steering_driver]
         /velocity_req               /steer_req
              │                           │
              └────────┬──────────────────┘
                       ▼
              [emulator_manager]   ◄──── Isaac Sim
               /vehicle/velocity_cmd      /Isaac/velocity_fb
               /vehicle/steering_cmd      /Isaac/steering_fb_1
                                          /Isaac/steering_fb_2
                       │
              ┌────────┴────────┐
              ▼                 ▼
       /velocity_cur       /steer_cur
              │                 │
       [motor_driver]    [steering_driver]
              │ /j1939_tx       │ /canopen_tx
              ▼                 ▼
       [j1939_parser]   [canopen_parser]
              └──────┬──────────┘
                     ▼
              [can_driver] ──CAN bus──► [VCU]
```

---

## 패키지 구성

| 패키지 | 설명 |
|---|---|
| `can_driver` | Linux SocketCAN 기반 CAN 프레임 송수신 (can0 / can1) |
| `j1939_parser` | J1939 프로토콜 파싱 및 빌드 |
| `canopen_parser` | CANopen 프로토콜 파싱 및 빌드 |
| `motor_driver` | VCU ↔ 모터 드라이버 간 토크·RPM 중계 |
| `steering_driver` | VCU ↔ 조향 드라이버 간 전류·AbsENC 중계 |
| `emulator_manager` | VCU 드라이버와 Isaac Sim 사이의 에뮬레이터 |
| `remote_controller` | 키보드 입력으로 VCU에 속도·조향 명령 전송 |
| `custom_msgs` | 패키지 간 공용 메시지 정의 |
| `issacSim_script` | Isaac Sim 내부에서 실행되는 AGV 제어·피드백 스크립트 |

---

## Emulator (`emulator_manager`)

VCU 드라이버 스택(motor_driver, steering_driver)과 Isaac Sim을 연결하는 브리지 노드입니다.

### 역할
- `motor_driver` / `steering_driver`가 계산한 **목표값(Req)** 을 Isaac Sim 포맷으로 변환하여 발행
- Isaac Sim에서 수신한 **현재값(FB)** 을 다시 드라이버 포맷으로 변환하여 반환

### 토픽

| 방향 | 토픽 | 타입 | 설명 |
|---|---|---|---|
| 구독 | `/velocity_req` | `VelocityReq` | motor_driver → 에뮬레이터 속도 요청 |
| 구독 | `/steer_req` | `SteerReq` | steering_driver → 에뮬레이터 조향각 요청 |
| 발행 | `/vehicle/velocity_cmd` | `Int16MultiArray` | 에뮬레이터 → Isaac Sim 속도 명령 (×100) |
| 발행 | `/vehicle/steering_cmd` | `Int16MultiArray` | 에뮬레이터 → Isaac Sim 조향 명령 (×10) |
| 구독 | `/Isaac/velocity_fb` | `Int16MultiArray` | Isaac Sim → 에뮬레이터 속도 피드백 (×100) |
| 구독 | `/Isaac/steering_fb_1` | `Int16MultiArray` | Isaac Sim → 에뮬레이터 조향 피드백 1축~2축 (×10) |
| 구독 | `/Isaac/steering_fb_2` | `Int16MultiArray` | Isaac Sim → 에뮬레이터 조향 피드백 3축~4축 (×10) |
| 발행 | `/velocity_cur` | `VelocityCur` | 에뮬레이터 → motor_driver 현재 속도 |
| 발행 | `/steer_cur` | `SteerCur` | 에뮬레이터 → steering_driver 현재 조향각 |

### Isaac Sim 스크립트

`issacSim_script/IsaacSim_Encoder_Feedback_with_VelocityFB.py`를 Isaac Sim 내부에서 실행합니다.  
8륜 AGV의 steer joint / drive joint를 직접 제어하며, 조향각 및 속도 피드백을 퍼블리시합니다.

---

## Remote Controller (`remote_controller`)

키보드 입력을 J1939 메시지로 변환하여 VCU에 **목표 속도 및 조향각**을 전송하는 노드입니다.

### 키 바인딩

| 키 | 동작 |
|---|---|
| `W` | 속도 증가 (+10) |
| `X` | 속도 감소 (-10) |
| `S` | 속도 중립 (127) |
| `A` | 조향 좌회전 (-10) |
| `D` | 조향 우회전 (+10) |
| `Q` | 좌회전 + 전진 |
| `E` | 우회전 + 전진 |
| `1` | Manual / Auto 모드 토글 |
| `2` | 전진 방향 설정 |
| `3` | 후진 방향 설정 |
| `4` | 일반 조향 / 크랩 조향 토글 |

- 속도 범위: 137 ~ 239 (중립 127)
- 조향 범위: 20 ~ 239 (중립 127)
- 발행 주기: 50 ms

### J1939 프로토콜

| 메시지 | PGN | SA | 내용 |
|---|---|---|---|
| RC Cmd → VCU | `0xF500` | `0xC8` | 속도(data[2]), 조향(data[0]) |
| RC State → VCU | `0xF501` | `0xC8` | Manual/Forward/Backward/Crab 비트 플래그 |

---

## 빌드 및 실행

### 요구사항

- ROS 2 Humble (또는 그 이상)
- Linux SocketCAN (CAN 인터페이스: `can0`, `can1`)
- NVIDIA Isaac Sim (에뮬레이터 연동 시)

### 빌드

```bash
cd ~/ros2_ws
colcon build --packages-select \
  custom_msgs can_driver j1939_parser canopen_parser \
  motor_driver steering_driver emulator_manager remote_controller
source install/setup.bash
```

### 전체 스택 실행 (Emulator 포함)

```bash
ros2 launch emulator_manager bringup.launch.py
```

launch 파일에 포함된 노드: `can0_node`, `can1_node`, `j1939_node`, `canopen_node`, `md_node`, `steering_node`, `emulator_node`

### Remote Controller 단독 실행

```bash
ros2 run remote_controller rc_node
```

---

## 커스텀 메시지

| 메시지 | 필드 | 설명 |
|---|---|---|
| `CanFrame` | id, dlc, data[8], is_extended | 원시 CAN 프레임 |
| `J1939Msg` | pgn, priority, src/dst, dlc, data[8] | J1939 파싱 결과 |
| `CanopenMsg` | function_code, node_id, dlc, data[8] | CANopen 파싱 결과 |
| `VelocityReq` | md1_velocity_req, md2_velocity_req | 모터 속도 요청 (m/s) |
| `VelocityCur` | md1_velocity_cur, md2_velocity_cur | 모터 현재 속도 (m/s) |
| `SteerReq` | steer_req[8] | 8륜 조향각 요청 (deg) |
| `SteerCur` | steer_cur[8] | 8륜 현재 조향각 (deg) |
| `RcCmd` | velocity, steer | RC 명령 원시값 |
