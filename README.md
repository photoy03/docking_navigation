# STT·SLAM 기반 어드미턴스 제어 도킹형 자율 안내 및 낙상예방 로봇

기존 수동 보행기에 자율주행 구동 모듈을 도킹하여 음성 명령으로 실내 목적지까지 안내하고, 사용자의 힘과 보행 의도를 반영해 주행을 보조하는 ROS 2 기반 스마트 보행 보조 로봇 프로젝트입니다.

본 시스템은 **Sherpa-ONNX 기반 음성인식**, **LiDAR SLAM / AMCL / Nav2 자율주행**, **Load Cell 기반 어드미턴스 제어**, **YOLO11n-Pose 기반 낙상 감지**, **STM32 기반 실시간 모터·안전 제어**를 통합합니다.

## 주요 기능

- Sherpa-ONNX 기반 온디바이스 STT 및 목적지 추론
- RPLIDAR A2M12 기반 SLAM Mapping
- AMCL 기반 위치 추정
- Nav2 기반 경로 계획 및 목적지 자율주행
- Load Cell + HX711 기반 어드미턴스 보행 보조
- YOLO11n-Pose 기반 낙상 감지
- STM32 상태머신 기반 제어권 분리
- Encoder + Stella AHRS 기반 Odometry
- Emergency Stop 및 Electronic Brake 연동

## 시스템 구성

### 상위 제어부

- NVIDIA Jetson Orin Nano
- Ubuntu 22.04 LTS
- ROS 2 Humble
- slam_toolbox
- AMCL / Nav2
- Sherpa-ONNX
- YOLO11n-Pose
- ROS 2 DDS

### 하위 제어부

- STM32 NUCLEO-F429ZI
- Load Cell 입력 처리
- 어드미턴스 제어
- Differential Drive 계산
- Encoder 기반 Wheel Velocity PID
- PWM / DIR 모터 제어
- Brake Relay 제어
- Emergency State 관리

### 주요 하드웨어

| Device | Purpose |
|---|---|
| RPLIDAR A2M12 | SLAM, AMCL, 장애물 인식 |
| Stella AHRS | 자세 및 Yaw 측정 |
| Load Cell + HX711 ×2 | 사용자 힘 및 하중 측정 |
| Logitech C920 | YOLO11n-Pose 영상 입력 |
| ReSpeaker 2-Mic Array | 음성 입력 |
| PG56-5898EB-2430 DC Motor ×2 | 좌·우 바퀴 구동 |
| Cytron MD30C R2 ×2 | DC Motor Driver |
| Electronic Brake | 정지 및 비상 제동 |

## 제어 상태

| State | Mode | Motor Control |
|---|---|---|
| `STATE_STOP` | 정지 / 비상정지 | PWM 0, Brake LOCK |
| `STATE_ADMITTANCE` | 보행 보조 | STM32 Load Cell 기반 제어 |
| `STATE_AUTO` | 자율주행 | Jetson Nav2 `/cmd_vel` 기반 제어 |

## ROS 2 Architecture

```text
User Voice
   ↓
Sherpa-ONNX STT
   ↓
Destination Resolver
   ↓
Nav2 NavigateToPose
   ↓
/cmd_vel
   ↓
mcu_bridge
   ↓
STM32
   ↓
Motor Driver
   ↓
PG56 Motors
```

보행 보조 모드:

```text
Load Cell
   ↓
HX711
   ↓
STM32
   ↓
Tare / Deadband
   ↓
Admittance Control
   ↓
Motor Command
```

낙상 감지:

```text
C920 Camera
   ↓
YOLO11n-Pose
   ↓
Fall Detection
   ↓
Emergency Command
   ↓
STM32 STATE_STOP
   ↓
PWM = 0 + Brake LOCK
```

## TF Structure

```text
map
 └── robot1_odom
      └── robot1_base
           └── laser
```

## Repository Structure

```text
docking_navigation/
├── fall_detection/     # YOLO11n-Pose 기반 낙상 감지
├── robot1_core/        # MCU bridge, odometry 및 핵심 제어
├── robot1_nav/         # AMCL / Nav2 / 목적지 주행
├── rplidar_ros/        # RPLIDAR ROS 2 Driver
├── slam/               # slam_toolbox 기반 Mapping
├── stella_ahrs/        # Stella AHRS ROS 2 Driver
├── voice_nav_pkg/      # Sherpa-ONNX STT 및 목적지 추론
└── .gitignore
```

주요 파일:

```text
robot1_core/
├── launch/odom_launch.py
├── src/mcu_bridge.cpp
└── src/odom_publisher.cpp

robot1_nav/
├── config/destinations.json
├── config/nav2_params.yaml
├── launch/navigation_launch.py
└── src/

slam/
└── launch/slam_launch.py

stella_ahrs/
└── launch/stella_ahrs_launch.py
```

## Development Environment

| Item | Environment |
|---|---|
| OS | Ubuntu 22.04 LTS |
| ROS | ROS 2 Humble |
| SBC | NVIDIA Jetson Orin Nano |
| MCU | STM32 NUCLEO-F429ZI |
| Languages | C++, Python, C |
| Build | colcon, CMake |
| Navigation | slam_toolbox, AMCL, Nav2 |
| Vision | OpenCV, YOLO11n-Pose |
| STT | Sherpa-ONNX |
| MCU Communication | USB Raw Serial |

> 본 프로젝트는 micro-ROS가 아닌 **Jetson–STM32 Raw Serial 통신 구조**를 사용합니다.

## Build

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/photoy03/docking_navigation.git

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install
source install/setup.bash
```

## 실행 예시

Stella AHRS:

```bash
ros2 launch stella_ahrs stella_ahrs_launch.py
```

Odometry:

```bash
ros2 launch robot1_core odom_launch.py
```

SLAM Mapping:

```bash
ros2 launch slam slam_launch.py
```

Navigation:

```bash
ros2 launch robot1_nav navigation_launch.py
```

실제 사용 시 LiDAR, IMU, STM32 Serial Port, Camera, Microphone 등이 올바르게 연결되어 있어야 합니다.

## Jetson–STM32 Serial Protocol

Jetson → STM32:

```text
M,0                  STOP
M,1                  ADMITTANCE
M,2                  AUTO
V,<linear>,<angular> Nav2 velocity command
E,1                  Emergency
PING
```

STM32 → Jetson:

```text
B,READY
B,TARE_OK
D,1
S,0 / S,1 / S,2
T,<left_total>,<right_total>
E,1,<reason>
PONG
```

## 주요 구동 파라미터

```text
Wheel Diameter : 0.216 m
Track Width    : 1.2405 m
Encoder CPR    : 3588

Max Linear     : 0.60 m/s
Max Angular    : 1.50 rad/s

Wheel PID
Kp             : 700
Ki             : 250
Kd             : 0
```

## 프로젝트 결과

- STT 기반 음성 목적지 설정
- LiDAR SLAM 기반 실내 지도 생성
- AMCL 기반 위치 추정
- Nav2 기반 목적지 자율주행
- Load Cell 기반 어드미턴스 보행 보조
- YOLO11n-Pose 기반 낙상 감지
- STM32 상태머신 기반 제어권 분리
- Encoder / IMU 기반 Odometry
- Emergency Stop 및 Electronic Brake 연동
- ROS 2 기반 Jetson–MCU 통합 제어

## Demo

프로젝트 동작 영상:

https://youtu.be/4EKiMY3l67k

## Project

**2026 한이음 드림업 프로젝트**

프로젝트명:  
**STT·SLAM 기반 어드미턴스 제어 도킹형 자율 안내 및 낙상예방 로봇**

주요 개발 분야:
- Autonomous Navigation
- Human-Robot Interaction
- Admittance Control
- Embedded Motor Control
- Vision AI
- ROS 2 System Integration

## Notes

이 repository는 프로젝트 최종 개발 버전을 기준으로 정리되었습니다.

센서 포트, Serial Device, 목적지 좌표 및 Nav2 Parameter는 실제 로봇 환경에 맞게 변경해야 할 수 있습니다.
