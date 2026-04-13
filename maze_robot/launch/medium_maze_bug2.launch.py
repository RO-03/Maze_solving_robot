"""
medium_maze_bug2.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name   = 'maze_robot'
    pkg_dir    = get_package_share_directory(pkg_name)
    world_path = os.path.join(pkg_dir, 'worlds', 'maze_medium.world')
    urdf_path  = os.path.join(pkg_dir, 'urdf',   'maze_robot.urdf.xacro')
    rviz_cfg   = os.path.join(pkg_dir, 'rviz',   'maze_robot.rviz')

    gz_sim = ExecuteProcess(cmd=['gz', 'sim', '-r', world_path], output='screen')

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/maze_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[('/model/maze_robot/tf', '/tf')],
        parameters=[{'use_sim_time': True}],
    )

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path]),
            'use_sim_time': True,
        }],
    )

    laser_tf = Node(
        package='tf2_ros', executable='static_transform_publisher', output='screen',
        arguments=[
            '--frame-id', 'laser_frame',
            '--child-frame-id', 'maze_robot/base_footprint/laser_sensor',
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
        ],
    )

    spawn = TimerAction(period=5.0, actions=[
        Node(
            package='ros_gz_sim', executable='create', output='screen',
            arguments=[
                '-string', Command(['xacro ', urdf_path]),
                '-name', 'maze_robot', '-world', 'maze_medium', '-z', '0.2',
            ],
        )
    ])

    slam = Node(
        package='slam_toolbox', executable='async_slam_toolbox_node',
        name='slam_toolbox', output='screen',
        parameters=[{
            'use_sim_time': True, 'odom_frame': 'odom',
            'map_frame': 'map', 'base_frame': 'base_footprint',
            'scan_topic': '/scan', 'mode': 'mapping',
            'resolution': 0.05, 'max_laser_range': 10.0,
            'minimum_travel_distance': 0.1, 'minimum_travel_heading': 0.1,
        }],
    )

    sensor = Node(package=pkg_name, executable='sensor_node',        output='screen',
                  parameters=[{'use_sim_time': True, 'noise_std_dev': 0.02,
                                'wall_enter_threshold': 0.38, 'front_cone_deg': 10}])
    algo   = Node(package=pkg_name, executable='bug2_baseline_node', output='screen',
                  parameters=[{'use_sim_time': True, 'goal_x': 0.0, 'goal_y': 6.0,
                                'map_frame': 'map', 'robot_frame': 'base_footprint',
                                'use_tf': True, 'max_linear_speed': 0.28,
                                'max_angular_speed': 1.2, 'kp_angular': 1.2,
                                'goal_radius': 0.4, 'stuck_timeout': 6.0}])
    logger = Node(package=pkg_name, executable='logger_node',        output='screen',
                  parameters=[{'use_sim_time': True, 'maze_name': 'medium'}])
    rviz   = Node(package='rviz2',   executable='rviz2',              output='screen',
                  arguments=['-d', rviz_cfg], parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        gz_sim, bridge, rsp, laser_tf, spawn, slam,
        TimerAction(period=8.0, actions=[sensor, algo, logger]),
        rviz,
    ])
