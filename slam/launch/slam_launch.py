import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # ==========================================================
    # 1. 패키지 경로 변수 선언 (절대 깨지지 않음)
    # ==========================================================
    robot1_core_dir = get_package_share_directory('robot1_core')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    slam_dir = get_package_share_directory('slam')
    rplidar_dir = get_package_share_directory('rplidar_ros')
    imu_dir = get_package_share_directory('stella_ahrs')

    # 파라미터 및 런치 파일 절대 경로
    robot1_config_file = os.path.join(robot1_core_dir, 'config', 'robot1_params.yaml')
    slam_config_file = os.path.join(slam_dir, 'config', 'mapper_params.yaml')
    slam_launch_file = os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')

    return LaunchDescription([
        # ==========================================================
        # 2. 하드웨어 드라이버 & 통신 노드 실행
        # ==========================================================
        # [A] 라이다 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(rplidar_dir, 'launch', 'rplidar_a1_launch.py'))
        ),

        # [B] IMU 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(imu_dir, 'launch', 'stella_ahrs_launch.py'))
        ),

        # [C] 🌟 아두이노 통신 브릿지 실행 (CMakeLists.txt 기준)
        Node(
            package='robot1_core',
            executable='mcu_bridge',  # CMake의 add_executable 이름과 정확히 일치!
            name='mcu_bridge_node',
            output='screen'
        ),

        # ==========================================================
        # 3. 우리가 만든 퓨전 오도메트리 노드 실행
        # ==========================================================
        # [D] 🌟 엔코더 + IMU 퓨전 오도메트리 (CMakeLists.txt 기준)
        Node(
            package='robot1_core',
            executable='odom_publisher',  # CMake의 add_executable 이름과 정확히 일치!
            name='robot1_odom_node',
            parameters=[robot1_config_file],
            # 주의: Odom C++ 코드는 "/robot1/imu/data"를 구독하지만, 
            # 실제 IMU는 "/imu/data"로 발행할 수 있으므로 안전하게 Remapping 해줍니다.
            remappings=[
                ('/robot1/imu/data', '/imu/data')
            ],
            output='screen'
        ),

        # ==========================================================
        # 4. TF 트리 연결 및 SLAM 실행
        # ==========================================================
        # [E] 라이다 위치 알려주기 (robot1_base -> laser)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0.1', '0', '0.2', '0', '0', '0', 'robot1_base', 'laser']
        ),

        # [F] SLAM Toolbox 메인 엔진 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_file),
            launch_arguments={'slam_params_file': slam_config_file}.items()
        )
    ])
