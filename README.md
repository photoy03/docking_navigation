AI 자율주행 기반 고령자 스마트 보행 보조 로봇
소개
이 프로젝트는 기존 수동 보행 보조차에 고성능 자율주행 모듈을 결합하여, 고령자의 보행 부담을 최소화하고 낙상 사고를 예방하며 목적지까지 안전하게 안내하는 융합형 스마트 보행 보조 로봇 시스템입니다. 사용자의 조작력을 정밀하게 계측하는 어드미턴스 제어와 ROS 2 기반의 실내 자율주행(SLAM/Nav2) 기술을 활용합니다.

주요 기능
어드미턴스 기반 반자율 보행 보조: 로드셀(Load Cell) 센서를 통해 조작력을 계측하고 로봇의 기계적 마찰을 상쇄하는 가변 토크를 제공하여 가벼운 조작감 구현

실시간 낙상 감지 및 예방: 비정상적인 체중 쏠림이나 급격한 하중 변화 감지 시, 즉각적인 모터 역토크 및 브레이크 체결로 낙상 사고 1차 예방

ROS 2 자율주행 에스코트: 2D LiDAR 및 뎁스 카메라를 활용한 SLAM 매핑과 Nav2 기반 경로 생성으로 주도권 충돌 없는 목적지 안내

고령자 친화적 음성 인터페이스: ReSpeaker 마이크 어레이를 활용한 음성 인식 및 목적지 설정 제어

사용 방법
워크스페이스 클론 및 빌드: Jetson Orin Nano(Ubuntu 24.04, ROS 2 Jazzy) 환경에서 깃허브 레포지토리를 다운로드하고 빌드합니다.

Bash
mkdir -p ~/smart_walker_ws/src
cd ~/smart_walker_ws/src
git clone [레포지토리 주소]
cd ~/smart_walker_ws
colcon build --symlink-install
하드웨어 제어 노드 실행: 아두이노 메가(하위 제어부)와의 통신을 위해 micro-ROS 에이전트를 실행합니다.

Bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
자율주행 런치 파일 실행: 메인 코어 노드와 자율주행 스택을 실행하여 시스템을 구동합니다.

Bash
ros2 launch smart_walker_core bringup.launch.py
라이선스
MIT License
