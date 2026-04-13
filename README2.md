# Maze-Solving Robot — ROS 2 Jazzy + Gazebo Harmonic

Autonomous frontier-based maze exploration using SLAM, a custom Bug2 controller, and nearest-frontier selection.

---

## Architecture Overview

```
Gazebo ──► ros_gz_bridge ──► /scan ──► sensor_node ──► /robot_state
                         ──► /odom ──►                             │
                         ──► /tf   ──► slam_toolbox ──► /map ──► frontier_node
                                                                        │
                                                               /goal_pose│
                                                                        ▼
                                                          hybrid_algorithm_node
                                                                   │
                                                          /cmd_vel  │
                         Gazebo ◄── ros_gz_bridge ◄────────────────┘
```

### Node Responsibilities

| Node | Role |
|---|---|
| `sensor_node` | Reads `/scan`, manages `EXPLORING ↔ WALL_FOLLOWING` state with hysteresis, publishes `/robot_state` |
| `frontier_node` | Reads `/map` from SLAM, vectorised frontier detection, nearest-frontier selection via TF2 map-frame pose, publishes `/goal_pose` |
| `hybrid_algorithm_node` | PD angular controller to goal, Bug2 wall-follower with M-line leave-point, TF2 map-frame pose, stuck recovery |
| `bug2_baseline_node` | Pure Bug2 (no leave-point) for performance comparison |
| `logger_node` | Records metrics to CSV, auto-generates comparison bar charts |
| `slam_toolbox` | Async online SLAM, publishes `/map` and `map → odom` TF |

---

## Prerequisites

```bash
# ROS 2 Jazzy
sudo apt install ros-jazzy-desktop

# Gazebo Harmonic
sudo apt install gz-harmonic

# ROS ↔ Gazebo bridge
sudo apt install ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim

# SLAM Toolbox
sudo apt install ros-jazzy-slam-toolbox

# TF2 Python bindings
sudo apt install ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs

# Python scientific stack
pip install numpy scipy matplotlib
```

---

## Build

```bash
# Clone / copy the package into your workspace
mkdir -p ~/ros2_ws/src
cp -r maze_robot ~/ros2_ws/src/

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select maze_robot
source install/setup.bash
```

---

## Run

### Autonomous Exploration (recommended)

```bash
# Simple maze
ros2 launch maze_robot simple_maze_hybrid.launch.py

# Medium maze
ros2 launch maze_robot medium_maze_hybrid.launch.py

# Complex maze
ros2 launch maze_robot complex_maze_hybrid.launch.py
```

### Pure Bug2 Baseline (for comparison)

```bash
ros2 launch maze_robot simple_maze_bug2.launch.py
ros2 launch maze_robot medium_maze_bug2.launch.py
ros2 launch maze_robot complex_maze_bug2.launch.py
```

---

## Key Improvements (vs MVP)

### 1. Coordinate Frame Fix (Critical)
`hybrid_algorithm_node` and `frontier_node` both use **TF2** (`map → base_footprint`) for robot position.  
No more odom-drift / frame mismatch between goal coordinates and robot position.

### 2. Nearest-Frontier Selection
`frontier_node` uses `scipy.ndimage` connected-component labelling and sorts frontier clusters by **Euclidean distance** from the robot. The closest frontier is always selected, eliminating the random back-tracking of the MVP.

### 3. Vectorised Frontier Detection
Frontier cells (UNKNOWN touching FREE) are found via `scipy.ndimage.maximum_filter` over the occupancy grid — a single numpy operation replacing `O(w×h)` Python loops. CPU usage is dramatically reduced on large maps.

### 4. PD Angular Controller
`hybrid_algorithm_node` uses a **proportional-derivative** angular controller (configurable `kp_angular` + `kd_angular`) for smoother turns and less oscillation when approaching goals.

### 5. Stuck Detection & Recovery
Both controller nodes detect zero-movement for `stuck_timeout` seconds and execute: 1 s reverse + 2 s random spin, then request a new goal from the frontier node.

### 6. Sensor Node Hysteresis
`sensor_node` requires N consecutive triggering scans before switching to `WALL_FOLLOWING`, preventing oscillation at threshold distance.

### 7. SLAM Integration
All launch files now include `slam_toolbox` (async online mapping). The `map → odom → base_footprint` TF chain is complete, enabling reliable map-frame navigation.

### 8. Exploration Complete Signal
`frontier_node` publishes `/exploration_complete (Bool)` when no frontiers remain. Both controller nodes listen and stop cleanly.

---

## Topic Reference

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Gazebo → ROS | LIDAR data |
| `/odom` | `nav_msgs/Odometry` | Gazebo → ROS | Wheel odometry |
| `/cmd_vel` | `geometry_msgs/Twist` | ROS → Gazebo | Velocity commands |
| `/tf` | `tf2_msgs/TFMessage` | Gazebo + RSP | Full TF tree |
| `/map` | `nav_msgs/OccupancyGrid` | slam_toolbox | Live occupancy grid |
| `/robot_state` | `std_msgs/String` | sensor_node | `EXPLORING` or `WALL_FOLLOWING` |
| `/goal_pose` | `geometry_msgs/PoseStamped` | frontier_node | Next navigation target |
| `/resume_exploring` | `std_msgs/Bool` | hybrid_node | M-line leave-point signal |
| `/goal_reached` | `std_msgs/Bool` | hybrid_node | Signals goal arrival to frontier |
| `/exploration_complete` | `std_msgs/Bool` | frontier_node | No frontiers left |
| `/path_trace` | `nav_msgs/Path` | controller | RViz path visualisation |
| `/performance_log` | `std_msgs/String` | controller | CSV metric string |

---

## Parameter Tuning

### sensor_node
| Parameter | Default | Effect |
|---|---|---|
| `wall_enter_threshold` | 0.38 m | Distance to trigger WALL_FOLLOWING |
| `wall_exit_threshold` | 0.55 m | (Hysteresis reference) |
| `front_cone_deg` | 10 | Half-width of front detection cone |
| `noise_std_dev` | 0.02 | Gaussian noise σ added to rays |

### frontier_node
| Parameter | Default | Effect |
|---|---|---|
| `goal_publish_rate` | 1.0 Hz | How often to re-evaluate frontiers |
| `min_frontier_size` | 8 cells | Ignore clusters smaller than this |
| `goal_arrival_radius` | 0.5 m | Consider goal reached within this distance |

### hybrid_algorithm_node / bug2_baseline_node
| Parameter | Default | Effect |
|---|---|---|
| `kp_angular` | 1.2 | Proportional angular gain |
| `kd_angular` | 0.08 | Derivative angular gain (hybrid only) |
| `max_linear_speed` | 0.28 m/s | Forward speed cap |
| `front_stop_dist` | 0.35 m | Start turning when obstacle within |
| `front_wedge_dist` | 0.22 m | Force zero linear speed |
| `stuck_timeout` | 6.0 s | Seconds before stuck recovery |

---

## Results

Performance metrics are saved to `$HOME/maze_robot_results/results/`:
- `experiment_results.csv` — raw data per run
- `performance_charts.png` — auto-generated bar charts (Time, Distance, Collisions)

---

## File Layout

```
maze_robot/
├── launch/
│   ├── simple_maze_hybrid.launch.py
│   ├── simple_maze_bug2.launch.py
│   ├── medium_maze_hybrid.launch.py
│   ├── medium_maze_bug2.launch.py
│   ├── complex_maze_hybrid.launch.py
│   └── complex_maze_bug2.launch.py
├── maze_robot/
│   ├── __init__.py
│   ├── sensor_node.py            ← state machine + scan processing
│   ├── frontier_node.py          ← NEW: nearest-frontier exploration
│   ├── hybrid_algorithm_node.py  ← Bug2 + M-line + TF2 + stuck recovery
│   ├── bug2_baseline_node.py     ← pure Bug2 comparison
│   └── logger_node.py            ← CSV + matplotlib charts
├── urdf/
│   └── maze_robot.urdf.xacro     ← fixed sensor frame_id + friction
├── worlds/
│   ├── maze_simple.world
│   ├── maze_medium.world
│   └── maze_complex.world
├── rviz/
│   └── maze_robot.rviz           ← Map + LaserScan + Path + GoalPose
├── package.xml                   ← all exec_depend declared
└── setup.py                      ← frontier_node entry point added
```
