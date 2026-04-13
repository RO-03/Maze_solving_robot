import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'maze_robot'
    pkg_dir = get_package_share_directory(pkg_name)

    world_path = os.path.join(pkg_dir, 'worlds', 'maze_simple.world')
    urdf_path  = os.path.join(pkg_dir, 'urdf', 'maze_robot.urdf.xacro')
    rviz_cfg   = os.path.join(pkg_dir, 'rviz', 'maze_robot.rviz')

    gz_sim = ExecuteProcess(cmd=['gz', 'sim', '-r', world_path], output='screen')

    # Bridge Gazebo <-> ROS  (include /clock so every node shares sim-time)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/maze_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[('/model/maze_robot/tf', '/tf')],
        parameters=[{'use_sim_time': True}]
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path]),
            'use_sim_time': True
        }]
    )

    # Static TF: connect RSP tree (laser_frame) to Gazebo sensor frame
    # New-style args to avoid deprecation warning
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[
            '--frame-id', 'laser_frame',
            '--child-frame-id', 'maze_robot/base_footprint/laser_sensor',
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0'
        ]
    )

    spawn = TimerAction(period=5.0, actions=[
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-string', Command(['xacro ', urdf_path]),
                '-name', 'maze_robot',
                '-world', 'maze_simple',
                '-z', '0.2'
            ]
        )
    ])

    sensor_node = Node(package=pkg_name, executable='sensor_node', output='screen',
                       parameters=[{'noise_std_dev': 0.02, 'use_sim_time': True}])

    algo_node = Node(package=pkg_name, executable='hybrid_algorithm_node', output='screen',
                     parameters=[{'goal_x': 0.0, 'goal_y': 4.0, 'use_sim_time': True}])

    logger = Node(package=pkg_name, executable='logger_node', output='screen',
                  parameters=[{'maze_name': 'simple', 'use_sim_time': True}])

    rviz = Node(package='rviz2', executable='rviz2', output='screen',
                arguments=['-d', rviz_cfg],
                parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        gz_sim, bridge, rsp, laser_tf, spawn,
        sensor_node, algo_node, logger, rviz
    ])
