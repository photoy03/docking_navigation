import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ==========================================================
    # Package directories
    # ==========================================================

    robot1_core_dir = get_package_share_directory(
        'robot1_core'
    )

    robot1_nav_dir = get_package_share_directory(
        'robot1_nav'
    )

    slam_toolbox_dir = get_package_share_directory(
        'slam_toolbox'
    )

    slam_dir = get_package_share_directory(
        'slam'
    )

    rplidar_dir = get_package_share_directory(
        'rplidar_ros'
    )

    imu_dir = get_package_share_directory(
        'stella_ahrs'
    )


    # ==========================================================
    # Config files
    # ==========================================================

    robot1_config_file = os.path.join(
        robot1_core_dir,
        'config',
        'robot1_params.yaml'
    )

    slam_config_file = os.path.join(
        slam_dir,
        'config',
        'mapper_params.yaml'
    )

    scan_filter_config = os.path.join(
        robot1_nav_dir,
        'config',
        'scan_filter.yaml'
    )

    slam_launch_file = os.path.join(
        slam_toolbox_dir,
        'launch',
        'online_async_launch.py'
    )

    rviz_config_file = os.path.join(
        slam_toolbox_dir,
        'config',
        'slam_toolbox_default.rviz'
    )


    # ==========================================================
    # Launch Description
    # ==========================================================

    return LaunchDescription([

        # ======================================================
        # 1. RPLIDAR A2M12
        # ======================================================

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    rplidar_dir,
                    'launch',
                    'rplidar_a2m12_launch.py'
                )
            )
        ),


        # ======================================================
        # 2. Stella AHRS
        # ======================================================

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    imu_dir,
                    'launch',
                    'stella_ahrs_launch.py'
                )
            )
        ),


        # ======================================================
        # 3. MCU Bridge
        # ======================================================

        Node(
            package='robot1_core',
            executable='mcu_bridge',
            name='mcu_bridge_node',
            output='screen'
        ),


        # ======================================================
        # 4. Encoder + IMU Odometry
        #
        # publishes:
        #   /robot1/odom
        #   robot1_odom -> robot1_base TF
        # ======================================================

        Node(
            package='robot1_core',
            executable='odom_publisher',
            name='robot1_odom_node',

            parameters=[
                robot1_config_file
            ],

            remappings=[
                ('/robot1/imu/data', '/imu/data')
            ],

            output='screen'
        ),


        # ======================================================
        # 5. robot1_base -> laser
        # ======================================================

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',

            arguments=[
                '--x', '0.189',
                '--y', '0.0',
                '--z', '0.0556',

                '--yaw', '3.14159265',
                '--pitch', '0.0',
                '--roll', '0.0',

                '--frame-id', 'robot1_base',
                '--child-frame-id', 'laser'
            ],

            output='screen'
        ),


        # ======================================================
        # 6. Laser Scan Filter
        #
        # /scan
        #   ↓
        # /scan_filtered
        #
        # SLAM과 이후 AMCL이 같은 scan source 사용
        # ======================================================

        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_to_scan_filter_chain',

            parameters=[
                scan_filter_config
            ],

            remappings=[
                ('scan', '/scan'),
                ('scan_filtered', '/scan_filtered')
            ],

            output='screen'
        ),


        # ======================================================
        # 7. RViz2
        # ======================================================

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',

            arguments=[
                '-d',
                rviz_config_file
            ],

            output='screen'
        ),


        # ======================================================
        # 8. SLAM Toolbox
        #
        # 중요:
        # LiDAR scan timestamp가 현재보다 약 0.1~0.17초
        # 과거이므로 odom TF history가 먼저 쌓이도록
        # SLAM만 1.5초 후 시작
        # ======================================================

        TimerAction(
            period=1.5,

            actions=[

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        slam_launch_file
                    ),

                    launch_arguments={
                        'slam_params_file':
                            slam_config_file,

                        'use_sim_time':
                            'false',

                        'autostart':
                            'true'
                    }.items()
                )

            ]
        ),

    ])
