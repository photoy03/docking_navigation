import os

from ament_index_python.packages import (
    get_package_share_directory
)

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess
)

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)

from launch.substitutions import (
    LaunchConfiguration
)

from launch_ros.actions import Node


def generate_launch_description():

    # ======================================================
    # Package paths
    # ======================================================

    nav_dir = get_package_share_directory(
        'robot1_nav'
    )

    robot1_core_dir = get_package_share_directory(
        'robot1_core'
    )

    rplidar_dir = get_package_share_directory(
        'rplidar_ros'
    )

    imu_dir = get_package_share_directory(
        'stella_ahrs'
    )
    
    nav2_bringup_dir = get_package_share_directory(
    'nav2_bringup'
    )


    # ======================================================
    # Config paths
    # ======================================================

    default_map = os.path.join(
        nav_dir,
        'maps',
        'my_map.yaml'
    )

    default_nav2_params = os.path.join(
        nav_dir,
        'config',
        'nav2_params.yaml'
    )
    
    scan_filter_params = os.path.join(
        nav_dir,
        'config',
        'scan_filter.yaml'
    )

    robot1_params = os.path.join(
        robot1_core_dir,
        'config',
        'robot1_params.yaml'
    )


    map_yaml = LaunchConfiguration(
        'map'
    )

    nav2_params = LaunchConfiguration(
        'params_file'
    )
    
    rviz_config = os.path.join(
    nav2_bringup_dir,
    'rviz',
    'nav2_default_view.rviz'
    )

    keepout_mask = os.path.join(
        nav_dir,
        'maps',
        'my_map_keepout.yaml'
    )
    # ======================================================
    # Lifecycle 순서
    #
    # configure / activate가 이 순서대로 진행됨
    # ======================================================

    # ============================================================
    # Lifecycle group 1: Localization
    # ============================================================
    localization_lifecycle_nodes = [
        'map_server',
        'keepout_filter_mask_server',
        'keepout_costmap_filter_info_server',
        'amcl',
    ]

    #============================================================
    # Lifecycle group 2: Navigation
    # ============================================================
    navigation_lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]


    return LaunchDescription([

        # ==================================================
        # Launch arguments
        # ==================================================

        DeclareLaunchArgument(
            'map',
            default_value=default_map
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=default_nav2_params
        ),


        # ==================================================
        # 1. RPLidar
        # ==================================================

        IncludeLaunchDescription(

            PythonLaunchDescriptionSource(

                os.path.join(
                    rplidar_dir,
                    'launch',
                    'rplidar_a2m12_launch.py'
                )
            ),

            launch_arguments={
                'serial_port': '/dev/rplidar'
            }.items()
        ),


        # ==================================================
        # 2. IMU
        # ==================================================

        IncludeLaunchDescription(

            PythonLaunchDescriptionSource(

                os.path.join(
                    imu_dir,
                    'launch',
                    'stella_ahrs_launch.py'
                )
            )
        ),


        # ==================================================
        # 3. STM32 Serial Bridge
        # ==================================================

        Node(
            package='robot1_core',

            executable='mcu_bridge',

            name='mcu_bridge_node',

            output='screen'
        ),


        # ==================================================
        # 4. Wheel + IMU Odometry
        # ==================================================

        Node(
            package='robot1_core',

            executable='odom_publisher',

            name='robot1_odom_node',

            parameters=[
                robot1_params
            ],

            remappings=[
                (
                    '/robot1/imu/data',
                    '/imu/data'
                )
            ],

            output='screen'
        ),


        # ==================================================
        # 5. Base → LiDAR static TF
        #
        # ==================================================
        
        
        Node(
            package='tf2_ros',

            executable='static_transform_publisher',

            name='base_to_laser_tf',

            arguments=[

                '--x', '0.189',
                '--y', '0.0',
                '--z', '0.0556',

                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '3.14159265',

                '--frame-id', 'robot1_base',
                '--child-frame-id', 'laser'
            ],

            output='screen'
        ),
        
        
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='scan_to_scan_filter_chain',

            parameters=[
                scan_filter_params
            ],

            remappings=[
                ('scan', '/scan'),
                ('scan_filtered', '/scan_filtered')
            ],

            output='screen'
        ),


        # ==================================================
        # 6. MAP SERVER
        # ==================================================

        Node(
            package='nav2_map_server',

            executable='map_server',

            name='map_server',

            parameters=[

                nav2_params,

                {
                    'yaml_filename':
                        map_yaml
                }
            ],

            output='screen'
        ),

        Node(
            package='nav2_map_server',
           executable='map_server',
            name='keepout_filter_mask_server',

            parameters=[
               nav2_params,
               {
                   'yaml_filename': keepout_mask
                }
            ],

            output='screen'
        ),


        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='keepout_costmap_filter_info_server',

            parameters=[
               nav2_params
            ],

            output='screen'
        ),
        # ==================================================
        # 7. AMCL
        # ==================================================

        Node(
            package='nav2_amcl',

            executable='amcl',

            name='amcl',

            parameters=[
                nav2_params
            ],

            output='screen'
        ),


        # ==================================================
        # 8. Controller
        #
        # Nav2 controller 출력은 바로 /cmd_vel로
        # 보내지 않고 velocity smoother로 전달
        # ==================================================

        Node(
            package='nav2_controller',

            executable='controller_server',

            name='controller_server',

            parameters=[
                nav2_params
            ],

            remappings=[

                (
                    'cmd_vel',
                    'cmd_vel_nav'
                )
            ],

            output='screen'
        ),


        # ==================================================
        # 9. Path smoother
        # ==================================================

        Node(
            package='nav2_smoother',

            executable='smoother_server',

            name='smoother_server',

            parameters=[
                nav2_params
            ],

            output='screen'
        ),


        # ==================================================
        # 10. Planner
        # ==================================================

        Node(
            package='nav2_planner',

            executable='planner_server',

            name='planner_server',

            parameters=[
                nav2_params
            ],

            output='screen'
        ),


        # ==================================================
        # 11. Behaviors
        # ==================================================

        Node(
            package='nav2_behaviors',

            executable='behavior_server',

            name='behavior_server',

            parameters=[
                nav2_params
            ],

            output='screen'
        ),


        # ==================================================
        # 12. Behavior Tree Navigator
        # ==================================================

        Node(
            package='nav2_bt_navigator',

            executable='bt_navigator',

            name='bt_navigator',

            parameters=[
                nav2_params
            ],

            output='screen'
        ),


        # ==================================================
        # 13. Waypoint follower
        # ==================================================

        Node(
            package='nav2_waypoint_follower',

            executable='waypoint_follower',

            name='waypoint_follower',

            parameters=[
                nav2_params
            ],

            output='screen'
        ),


        # ==================================================
        # 14. Velocity smoother
        #
        # controller
        #    ↓
        # cmd_vel_nav
        #    ↓
        # velocity_smoother
        #    ↓
        # /cmd_vel
        #    ↓
        # mcu_bridge
        #    ↓
        # STM32
        # ==================================================

        Node(
            package='nav2_velocity_smoother',

            executable='velocity_smoother',

            name='velocity_smoother',

            parameters=[
                nav2_params
            ],

            remappings=[

                (
                    'cmd_vel',
                    'cmd_vel_nav'
                ),

                (
                    'cmd_vel_smoothed',
                    'cmd_vel'
                )
            ],

            output='screen'
        ),

        # ==================================================
        # 15. Destination Resolver
        #
        # /voice_command
        #      ↓
        # destinations.json
        #      ↓
        # /destination_pose
        # ==================================================

        Node(
            package='robot1_nav',
            executable='destination_resolver',
            name='destination_resolver',
            output='screen'
        ),

        Node(
            package='robot1_nav',
            executable='mode_manager',
            name='mode_manager',
            output='screen'
        ),

        # ==================================================
        # 16. Nav2 Goal Client
        #
        # /destination_pose
        #      ↓
        # /navigate_to_pose
        # ==================================================

        Node(
            package='robot1_nav',
            executable='nav_goal_client',
            name='nav_goal_client',
            output='screen'
        ),
        
        
        Node(
    	    package='voice_nav_pkg',
    	    executable='voice_text_publisher',
    	    name='voice_text_publisher',
    	    output='screen'
	),

        
                # ==================================================
        # Localization Startup Gate
        #
        # /mcu/docked = true
        #        ↓
        # lifecycle_manager_localization STARTUP
        # ==================================================

        Node(
            package='robot1_nav',

            executable='localization_startup_gate',

            name='localization_startup_gate',

            output='screen'
        ),


        # ==================================================
        # Localization Ready Gate
        #
        # scan + odom + AMCL + TF 확인
        #          ↓
        # Navigation Lifecycle STARTUP
        # ==================================================

        Node(
            package='robot1_nav',

            executable='localization_ready_gate',

            name='localization_ready_gate',

            output='screen'
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d',
                rviz_config
            ],
            parameters=[
                {'use_sim_time': False}
            ],
            output='screen'
        ),

        # ==================================================
        # Fall Detection
        #
        # C920
        #   ↓
        # YOLO11 Pose
        #   ↓
        # /fall/detected
        #   ↓
        # /mcu/emergency_cmd
        #   ↓
        # mcu_bridge
        #   ↓
        # STM32 Emergency
        #
        # YOLO / Torch는 별도 fall_venv에서 실행
        # ==================================================

        TimerAction(
            period=3.0,

            actions=[
                ExecuteProcess(
                    cmd=[
                        '/home/jet/fall_venv/bin/python',

                        '/home/jet/ros2_ws/src/fall_detection/'
                        'fall_detection/fall_detection_node.py'
                    ],

                    output='screen'
                )
            ]
        ),

        ## ==================================================
        # 17. Lifecycle Managers
        #
        # 1단계:
        # Localization lifecycle은 자동 시작
        #
        # 2단계:
        # Navigation lifecycle은 실행만 해두고
        # localization 준비 확인 후 별도로 STARTUP
        # ==================================================

        TimerAction(

            period=3.0,

            actions=[

                # ==========================================
                # Localization Lifecycle Manager
                # ==========================================

                Node(
                    package='nav2_lifecycle_manager',

                    executable='lifecycle_manager',

                    name='lifecycle_manager_localization',

                    parameters=[{

                        'use_sim_time': False,

                        'autostart': False,

                        'bond_timeout': 4.0,

                        'node_names':
                            localization_lifecycle_nodes
                    }],

                    output='screen'
                ),


                # ==========================================
                # Navigation Lifecycle Manager
                # ==========================================

                Node(
                    package='nav2_lifecycle_manager',

                    executable='lifecycle_manager',

                    name='lifecycle_manager_navigation',

                    parameters=[{

                        'use_sim_time': False,

                        'autostart': False,

                        'bond_timeout': 4.0,

                        'node_names':
                            navigation_lifecycle_nodes
                    }],

                    output='screen'
                )
            ]
        )
    ])
