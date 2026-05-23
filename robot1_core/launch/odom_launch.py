import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # robot1_core 패키지의 share 경로에서 YAML 파일 위치 획득
    config_dir = os.path.join(
        get_package_share_directory('robot1_core'),
        'config',
        'robot1_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='robot1_core',
            executable='odom_publisher',
            name='robot1_odom_node',
            parameters=[config_dir],   # 변수 지정 지시
            output='screen'
        )
    ])
