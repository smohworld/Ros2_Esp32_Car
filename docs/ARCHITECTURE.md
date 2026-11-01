# System Architecture

ESP32 기반 4륜 RC카를 **micro-ROS(Wi-Fi/UDP)** 로 ROS 2 Humble에 연결하고,
PC에서 **SLAM → AMCL/Nav2 자율주행 → 라이다 기반 주차 → YOLOv8 장난감 인식/추종**을
하나의 미션으로 묶어 실행하는 시스템입니다.

---

## 1. 전체 구성도

```mermaid
flowchart LR
    subgraph HW["🚗 RC Car (On-board)"]
        direction TB
        LIDAR["LD14P LiDAR<br/>360° · 6Hz · 4kHz"]
        IMU["MPU6050<br/>Gyro Z"]
        ENC["Encoder x4"]
        MOT["모터드라이버 + DC 기어모터 x4<br/>(Skid-steer)"]
        BAT["배터리 전압 (ADC)"]

        subgraph ESP32["WeMos D1 R32 · ESP32-WROOM-32 (micro-ROS client)"]
            direction TB
            PARSE["LiDAR UART Parser<br/>(analysis_one)"]
            CTRL["RobotController<br/>PID 4ch · 100Hz"]
            UROS["micro-ROS rclc<br/>executor"]
        end

        CAM["ESP32-CAM<br/>MJPEG HTTP Server"]

        LIDAR -- UART --> PARSE
        IMU -- I2C --> CTRL
        ENC --> CTRL
        BAT -- ADC --> CTRL
        CTRL -- PWM --> MOT
        PARSE --> UROS
        CTRL <--> UROS
    end

    subgraph PC["💻 Host PC (Ubuntu 22.04 · ROS 2 Humble)"]
        direction TB
        AGENT["micro_ros_agent<br/>udp4 :8888"]

        subgraph BRINGUP["robot_bringup"]
            TEL["robot_telemetry_publisher<br/>엔코더+자이로 → odom/TF"]
            LP["lidar_publisher<br/>세그먼트 → LaserScan"]
            RSP["robot_state_publisher<br/>URDF"]
        end

        CAMPUB["mjpeg_camera_publisher"]

        subgraph NAV["Navigation"]
            SLAM["slam_toolbox<br/>(맵 생성)"]
            AMCL["AMCL + map_server"]
            NAV2["Nav2<br/>NavFn + RotationShim(DWB)"]
        end

        subgraph VISION["Vision"]
            DET["toy_detector<br/>YOLOv8n"]
            TRK["toy_tracker<br/>시각 서보 추종"]
        end

        subgraph APP["Mission Layer"]
            GUI["mission_gui (PyQt5)"]
            RUN["MissionRunner"]
            PARK["Parker<br/>RANSAC 벽 검출 주차"]
            LOC["Localizer<br/>회전 전역 위치추정"]
            RI["RobotInterface<br/>(공용 ROS 노드)"]
        end
    end

    UROS <== "Wi-Fi UDP<br/>XRCE-DDS" ==> AGENT
    CAM == "HTTP MJPEG<br/>:80/stream" ==> CAMPUB

    AGENT -- "/encoder_deltas_gyroz_bat_rssi" --> TEL
    AGENT -- "/range_segment" --> LP
    TEL -- "/odom, /tf(odom→base_link)" --> NAV
    LP -- "/scan" --> NAV
    LP -- "/scan" --> PARK
    CAMPUB -- "/camera/image_raw" --> DET
    DET -- "/toy_detection" --> TRK
    DET -- "/toy_detection" --> RUN
    GUI --> RUN
    RUN --> PARK & LOC
    RUN & PARK & LOC --> RI
    RI -- "NavigateToPose" --> NAV2
    RI -- "/toy_tracker/enable" --> TRK
    NAV2 -- "/cmd_vel" --> AGENT
    TRK -- "/cmd_vel" --> AGENT
    RI -- "/cmd_vel (주차·회전)" --> AGENT
```

---

## 2. 계층 구조

| 계층 | 구성 요소 | 역할 |
|---|---|---|
| **Firmware** | WeMos D1 R32 / ESP32-WROOM-32 (`esp32/ros2_rc_car/`) | 모터 PID 제어, 엔코더·IMU 수집, LiDAR UART 파싱, micro-ROS 통신 |
| | ESP32-CAM (`esp32/camera/`) | OV3660 카메라 영상을 MJPEG HTTP 스트림으로 송출 (고정 IP) |
| **Bridge** | `micro_ros_agent`, `mjpeg_camera_publisher` | 마이크로컨트롤러/HTTP 데이터를 ROS 2 토픽으로 변환 |
| **Driver** | `robot_telemetry_publisher`, `lidar_publisher`, `robot_state_publisher` | 원시 데이터를 표준 메시지(`Odometry`, `LaserScan`, `TF`)로 변환 |
| **Navigation** | `slam_toolbox`, `AMCL`, `Nav2` | 지도 작성, 위치 추정, 경로 계획 및 추종 |
| **Perception** | `toy_detector`, `toy_tracker` | YOLOv8 객체 인식, 카메라 기반 시각 서보 |
| **Application** | `mission_gui`, `MissionRunner`, `Parker`, `Localizer` | 미션 편집·실행, 정밀 주차, 전역 위치 추정 |

---

## 3. ESP32 펌웨어

```mermaid
flowchart TB
    subgraph LOOP["loop()  — 단일 스레드 협조형 스케줄링"]
        A["① LiDAR UART FIFO 블록 읽기<br/>AnalysisOne() 프레임 동기화 · CRC"] --> B{"56 패킷 누적?<br/>(≈1회전)"}
        B -- yes --> C["720 포인트를 4개 세그먼트로 분할<br/>[sec|nsec|idx|uint16×180] publish"]
        B -- no --> D
        C --> D["② 10ms 주기 도달?"]
        D -- yes --> E["updateMotorSpeedsRamped(dt)<br/>4륜 PID + 가감속 램프"]
        E --> F["Int64MultiArray publish<br/>[sec,nsec,FL,RL,FR,RR,gz,bat,rssi]"]
        D -- no --> G
        F --> G["③ rclc_executor_spin_some(1ms)<br/>/cmd_vel 콜백 → setTargetSpeeds(v,w)"]
        G --> H["④ 1초마다 상태 점검<br/>Wi-Fi·Agent ping 실패 시 ESP.restart()"]
    end
```

| 모듈 | 파일 | 설명 |
|---|---|---|
| 모터 제어 | `DCMotorPWM`, `DCMotorSpeed`, `PIDController` | 바퀴별 RPM 피드백 PID, PWM 출력 |
| 차량 제어 | `RobotController` | `(v, ω)` → 좌/우 RPM 변환(차동 구동), 4륜 동시 제어, 배터리 측정 |
| 센서 | `Encoder`, `myMPU6050` | 4채널 쿼드러처 엔코더, 자이로 Z축 (캘리브레이션 포함) |
| LiDAR | `analysis_one`, `lidar_frame.h`, `lidar_publish` | LD14P 프레임(0x54 헤더, 12pt/pack) 파싱 및 세그먼트 발행 |
| 통신 | `rosCmdVel`, `rosEncoderDeltasGyroZ` | micro-ROS 노드 / 구독·발행자 초기화 |

**설계 포인트**
- **세그먼트 분할 전송**: micro-ROS XRCE-DDS의 MTU 제한 때문에 1회전(720pt × 2B)을 4개 메시지로 나누어 보내고 PC에서 재조립합니다.
- **NTP 타임스탬프**: 엔코더·LiDAR 데이터에 ESP32 측 실제 측정 시각을 넣어 PC에서 `dt`를 정확하게 계산합니다.
- **자가 복구**: Wi-Fi 끊김이나 Agent 무응답을 감지하면 재부팅해 자동으로 재연결합니다.

---

## 4. ROS 2 토픽 / TF

### 주요 토픽

| Topic | Type | Publisher → Subscriber |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 · toy_tracker · RobotInterface → ESP32 |
| `/encoder_deltas_gyroz_bat_rssi` | `std_msgs/Int64MultiArray` | ESP32 → robot_telemetry_publisher |
| `/range_segment` | `std_msgs/UInt8MultiArray` | ESP32 → lidar_publisher |
| `/odom` | `nav_msgs/Odometry` | robot_telemetry_publisher → Nav2 |
| `/joint_states` | `sensor_msgs/JointState` | robot_telemetry_publisher → robot_state_publisher |
| `/battery_voltage`, `/rssi_strength` | `Float32`, `Int32` | robot_telemetry_publisher → GUI |
| `/scan` | `sensor_msgs/LaserScan` | lidar_publisher → SLAM · AMCL · Nav2 · Parker |
| `/camera/image_raw`, `/camera/camera_info` | `Image`, `CameraInfo` | mjpeg_camera_publisher → toy_detector |
| `/toy_detection` | `Float32MultiArray` `[found,cx,cy,w,h,conf]` | toy_detector → toy_tracker · MissionRunner |
| `/toy_detection/image` | `sensor_msgs/Image` | toy_detector → mission_gui |
| `/toy_tracker/enable` | `std_msgs/Bool` | RobotInterface → toy_tracker |
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | RobotInterface → Nav2 |

### TF 트리

```mermaid
flowchart TB
    map -->|AMCL / slam_toolbox| odom
    odom -->|robot_telemetry_publisher| base_link
    base_link --> base_footprint
    base_link --> chassis
    chassis --> front_left_wheel & rear_left_wheel & front_right_wheel & rear_right_wheel
    base_link --> laser_frame
    base_link --> camera_link
    base_link --> controller_frame
```

---

## 5. 미션 실행 흐름

미션은 `maps/<map>_mission.json`에 장소(`places`), 주차 구역(`parking`), 단계 목록(`mission`)으로 저장되고,
GUI에서 편집하거나 `mission_runner`로 GUI 없이 실행할 수 있습니다.

```mermaid
sequenceDiagram
    autonumber
    participant G as mission_gui
    participant M as MissionRunner
    participant L as Localizer
    participant N as Nav2
    participant P as Parker
    participant D as toy_detector
    participant T as toy_tracker

    G->>M: run(mission.json)
    M->>L: localize(turns)
    L->>L: 전역 파티클 초기화 → 45° 단위 제자리 회전<br/>AMCL 공분산·스캔 일치율로 수렴 판정
    M->>N: NavigateToPose(move_to_parking)
    N-->>M: succeeded
    M->>P: park(parking1)
    P->>P: /scan RANSAC 벽 검출 → pre-park 자세 이동<br/>→ 재측정 → odom 고정 최종 접근
    M->>N: NavigateToPose(find_toy)
    M->>D: search_toy — 30°씩 회전하며 conf≥0.9 3회 연속 확인
    M->>T: /toy_tracker/enable = true
    T->>T: cx·박스높이 기반 시각 서보 (30cm 유지)
    M-->>T: toy_goal 반경 진입 시 enable = false
    M->>N: NavigateToPose(parking1)
    M->>P: park(parking1)
    M-->>G: Mission complete
```

| Step type | 설명 | 사용 모듈 |
|---|---|---|
| `localize` | 제자리 회전으로 AMCL 전역 위치 수렴 | `localization.py` |
| `nav` | 지정 장소/주차 구역으로 이동 | Nav2 `NavigateToPose` |
| `park` | LiDAR 벽 검출 기반 정밀 주차 | `parking.py` |
| `search_toy` | 회전하며 YOLO로 장난감 탐색 | `toy_detector` |
| `follow` | 목표 지점 반경 안에 들어올 때까지 장난감 추종 | `toy_tracker` |
| `final` | Nav2로 주차 구역 근처까지 이동한 뒤 주차 | Nav2 + `parking.py` |

---

## 6. 런치 구성

```mermaid
flowchart LR
    MISSION["mission.launch.py"] --> BR["robot_bringup.launch.py"]
    MISSION --> NV["navigation.launch.py<br/>(+2s)"]
    MISSION --> CAMN["mjpeg_camera_publisher"]
    MISSION --> DETN["toy_detector --no-view"]
    MISSION --> TRKN["toy_tracker --start-disabled"]
    MISSION --> GUIN["mission_gui (+6s)"]
    BR --> RSPL["rsp.launch.py"]
    BR --> AG["micro_ros_agent"]
    BR --> TELN["robot_telemetry_publisher"]
    BR --> LPN["lidar_publisher"]
    NV --> LOCL["nav2 localization_launch<br/>(map_server + AMCL)"]
    NV --> NAVL["nav2 navigation_launch (+3s)"]
    SL["slam.launch.py"] --> ST["slam_toolbox online_async"]
    SL --> NAVL2["nav2 navigation_launch (+3s)"]
```

---

## 7. 네트워크

```
                ┌──────────────── Wi-Fi (Windows 모바일 핫스팟 192.168.137.0/24) ────────────────┐
                │                                                                               │
   D1 R32 (DHCP) ─ UDP :8888 (micro-ROS XRCE-DDS) ──▶  Host PC 192.168.137.1  (micro_ros_agent)  │
   ESP32-CAM 192.168.137.100 ── HTTP :80/stream ────▶  Host PC (mjpeg_camera_publisher)          │
                └───────────────────────────────────────────────────────────────────────────────┘
```
