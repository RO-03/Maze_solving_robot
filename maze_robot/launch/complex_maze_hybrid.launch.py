"""
complex_maze_hybrid.launch.py
================================
Full autonomous exploration launch for the complex maze:
  • Gazebo Harmonic simulation
  • ros_gz_bridge (clock, scan, cmd_vel, odom, tf, joint_states)
  • robot_state_publisher  (URDF → /tf)
  • static TF for laser frame
  • slam_toolbox (async online SLAM → /map)
  • sensor_node   (state machine)
  • frontier_node (nearest-frontier goal publisher)
  • hybrid_algorithm_node (Bug2 + M-line controller)
  • logger_node
  • rviz2
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name  = 'maze_robot'
    pkg_dir   = get_package_share_directory(pkg_name)
    world_path = os.path.join(pkg_dir, 'worlds', 'maze_complex.world')
    urdf_path  = os.path.join(pkg_dir, 'urdf',   'maze_robot.urdf.xacro')
    rviz_cfg   = os.path.join(pkg_dir, 'rviz',   'maze_robot.rviz')

    # ── Gazebo simulation ─────────────────────────────────────────────────
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen',
    )

    # ── ROS ↔ Gazebo bridge ───────────────────────────────────────────────
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
        parameters=[{'use_sim_time': True}],
    )

    # ── Robot state publisher (URDF → /tf) ───────────────────────────────
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path]),
            'use_sim_time': True,
        }],
    )

    # ── Static TF: laser_frame → Gazebo sensor link ───────────────────────
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=[
            '--frame-id',       'laser_frame',
            '--child-frame-id', 'maze_robot/base_footprint/laser_sensor',
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
        ],
    )

    # ── Spawn robot (delayed to let Gazebo fully start) ───────────────────
    spawn = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-string', Command(['xacro ', urdf_path]),
                    '-name',   'maze_robot',
                    '-world',  'maze_complex',
                    '-z',      '0.2',
                ],
            )
        ],
    )

    # ── SLAM Toolbox (async online – builds /map live) ────────────────────
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time':          True,
            'odom_frame':            'odom',
            'map_frame':             'map',
            'base_frame':            'base_footprint',
            'scan_topic':            '/scan',
            'mode':                  'mapping',
            'resolution':            0.05,
            'max_laser_range':       10.0,
            'minimum_travel_distance': 0.1,
            'minimum_travel_heading':  0.1,
        }],
    )

    # ── Sensor node (state machine: EXPLORING ↔ WALL_FOLLOWING) ──────────
    sensor = Node(
        package=pkg_name,
        executable='sensor_node',
        output='screen',
        parameters=[{
            'use_sim_time':        True,
            'noise_std_dev':       0.02,
            'add_noise':           True,
            'wall_enter_threshold': 0.38,
            'wall_exit_threshold':  0.55,
            'front_cone_deg':       10,
            'publish_rate':         20.0,
        }],
    )

    # ── Frontier node (nearest-frontier goal publisher) ───────────────────
    frontier = Node(
        package=pkg_name,
        executable='frontier_node',
        output='screen',
        parameters=[{
            'use_sim_time':        True,
            'goal_publish_rate':   1.0,
            'min_frontier_size':   8,
            'goal_arrival_radius': 0.5,
            'map_frame':           'map',
            'robot_frame':         'base_footprint',
            'use_tf':              True,
        }],
    )

    # ── Hybrid controller (Bug2 + M-line + TF2 pose) ─────────────────────
    algo = Node(
        package=pkg_name,
        executable='hybrid_algorithm_node',
        output='screen',
        parameters=[{
            'use_sim_time':        True,
            'map_frame':           'map',
            'robot_frame':         'base_footprint',
            'use_tf':              True,
            'max_linear_speed':    0.28,
            'max_angular_speed':   1.2,
            'kp_angular':          1.2,
            'kd_angular':          0.08,
            'goal_radius':         0.45,
            'front_stop_dist':     0.35,
            'front_wedge_dist':    0.22,
            'side_target_dist':    0.35,
            'side_tol':            0.08,
            'stuck_timeout':       6.0,
            'stuck_move_tol':      0.05,
        }],
    )

    # ── Logger node ───────────────────────────────────────────────────────
    logger = Node(
        package=pkg_name,
        executable='logger_node',
        output='screen',
        parameters=[{'use_sim_time': True, 'maze_name': 'complex'}],
    )

    # ── RViz2 ────────────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        gz_sim, bridge, rsp, laser_tf, spawn,
        slam,
        # Delay custom nodes slightly to let SLAM initialise
        TimerAction(period=8.0, actions=[sensor, frontier, algo, logger]),
        rviz,
    ])
