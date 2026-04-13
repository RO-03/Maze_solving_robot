# Autonomous Maze-Solving Mobile Robot
### ROS2 Jazzy | Gazebo Harmonic | Ubuntu 24.04

A ROS2-based simulation of a differential-drive mobile robot that autonomously navigates three maze environments using a **Hybrid Bug2 + Frontier Exploration** algorithm, benchmarked against a plain Bug2 baseline.

---

## 📋 Table of Contents
- [Algorithm Overview](#algorithm-overview)
- [Project Structure](#project-structure)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Running the Simulation](#running-the-simulation)
- [RViz Visualization](#rviz-visualization)
- [Performance Results](#performance-results)
- [Node Architecture](#node-architecture)
- [Team](#team)

---

## 🧠 Algorithm Overview

### Hybrid Bug2 + Frontier Exploration (our unique approach)

| Phase | Behaviour |
|-------|-----------|
| **EXPLORING** | Robot drives straight toward the goal along the M-line (imaginary line from start → goal) |
| **WALL_FOLLOWING** | Triggered when front sensor < 0.4 m; robot hugs the left wall (Bug2 left-hand rule) |
| **Return to M-line** | When the robot crosses the M-line again AND is closer to the goal than the hit-point, it breaks away from the wall and resumes EXPLORING |
| **Goal Detection** | Stops when within 0.3 m of the goal coordinates |

**Why this beats plain Bug2:** The hybrid dynamically escapes wall-following as soon as it's productive, instead of tracing the entire wall contour. This yields shorter paths on medium and complex mazes.

### Option C Architecture (state coordination)
```
sensor_node ──/robot_state──► hybrid_algorithm_node
     ▲                               │
     └──────── /resume_exploring ────┘
```
- `sensor_node` monitors `/scan` and publishes `EXPLORING` or `WALL_FOLLOWING` to `/robot_state`
- `hybrid_algorithm_node` handles M-line math; when escape conditions are met it publishes `True` to `/resume_exploring`, telling `sensor_node` to flip back to EXPLORING

---

## 📁 Project Structure

```
MAR_project/
└── maze_robot/
    ├── maze_robot/
    │   ├── sensor_node.py          # Reads /scan, adds Gaussian noise, publishes /robot_state
    │   ├── hybrid_algorithm_node.py# Hybrid Bug2+Frontier core logic
    │   ├── bug2_baseline_node.py   # Plain Bug2 (for comparison)
    │   └── logger_node.py          # Records metrics, generates matplotlib charts
    ├── urdf/
    │   └── maze_robot.urdf.xacro   # Differential-drive robot model
    ├── worlds/
    │   ├── maze_simple.world       # Easy  – 4-6 walls,  goal at (0, 4)
    │   ├── maze_medium.world       # Medium – 11 walls,  goal at (0, 8.5)
    │   └── maze_complex.world      # Hard   – 22+ walls, goal at (0, 8)
    ├── rviz/
    │   └── maze_robot.rviz         # Pre-configured RViz layout
    ├── launch/
    │   ├── simple_maze_hybrid.launch.py
    │   ├── simple_maze_bug2.launch.py
    │   ├── medium_maze_hybrid.launch.py
    │   ├── medium_maze_bug2.launch.py
    │   ├── complex_maze_hybrid.launch.py
    │   └── complex_maze_bug2.launch.py
    ├── results/                    # Auto-generated CSV + PNG charts appear here
    ├── package.xml
    └── setup.py
```

---

## ✅ Prerequisites

| Dependency | Version |
|------------|---------|
| Ubuntu | 24.04 (Noble) |
| ROS2 | Jazzy Jalisco |
| Gazebo | Harmonic (ships with Jazzy) |
| Python | 3.12 |
| `ros_gz_bridge` | Jazzy |
| `ros_gz_sim` | Jazzy |
| `xacro` | Jazzy |
| `matplotlib` | Any recent |

Install missing ROS2 packages if needed:
```bash
sudo apt install ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim \
                 ros-jazzy-xacro ros-jazzy-robot-state-publisher \
                 python3-matplotlib
```

---

## 🚀 Installation

```bash
# 1. Clone the repository
git clone https://github.com/<your-username>/MAR_project.git
cd MAR_project

# 2. Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# 3. Build the workspace
colcon build --symlink-install

# 4. Source the workspace overlay
source install/setup.bash
```

---

## ▶️ Running the Simulation

Each of the 6 launch files starts **Gazebo + RViz + all nodes** in one command.

### Hybrid Algorithm runs
```bash
ros2 launch maze_robot simple_maze_hybrid.launch.py
ros2 launch maze_robot medium_maze_hybrid.launch.py
ros2 launch maze_robot complex_maze_hybrid.launch.py
```

### Plain Bug2 baseline runs (for comparison)
```bash
ros2 launch maze_robot simple_maze_bug2.launch.py
ros2 launch maze_robot medium_maze_bug2.launch.py
ros2 launch maze_robot complex_maze_bug2.launch.py
```

> **Tip:** Run all 6 scenarios back-to-back. Each run appends to `results/experiment_results.csv` and regenerates `results/performance_charts.png` automatically.

### Noise parameter
Gaussian noise on the laser sensor can be tuned via the `noise_std_dev` parameter (default `0.02 m`):
```bash
ros2 launch maze_robot simple_maze_hybrid.launch.py noise_std_dev:=0.05
```

---

## 🖥️ RViz Visualization

The pre-configured RViz layout shows:

| Panel | Topic | Description |
|-------|-------|-------------|
| **LaserScan** | `/scan` | 360° sensor rays rendered as green points |
| **Path** | `/path_trace` | Red breadcrumb trail of robot trajectory |
| **RobotModel** | `/robot_description` | 3D robot mesh |
| **Grid** | — | Reference ground plane |

The terminal output shows real-time mode switches:
```
[sensor_node]: Wall detected at 0.39m → WALL_FOLLOWING
[hybrid_algorithm_node]: Left Point reached! Resuming M-Line.
[hybrid_algorithm_node]: GOAL REACHED!
```

---

## 📊 Performance Results

After running all 6 scenarios, open the auto-generated chart:
```bash
xdg-open results/performance_charts.png
```

Raw data is saved at:
```
results/experiment_results.csv   # Maze, Algorithm, Time, PathLength, Collisions, ModeSwitches
```

### Sample results (Simple Maze)
| Algorithm | Time (s) | Path Length (m) |
|-----------|----------|-----------------|
| Hybrid Bug2 | 122.1 | 20.1 |
| Plain Bug2  | 73.4  | 11.4 |

> The Hybrid algorithm performs significantly better on **medium and complex mazes** where escaping the wall early saves substantial distance. The simple maze has such a short direct path that Bug2's wall-tracing is occasionally faster.

---

## 🔌 Node Architecture

### Topic Interface Contract

| Topic | Type | Publisher | Subscriber |
|-------|------|-----------|------------|
| `/scan` | `sensor_msgs/LaserScan` | Gazebo (bridge) | `sensor_node` |
| `/robot_state` | `std_msgs/String` | `sensor_node` | `hybrid_algorithm_node` |
| `/resume_exploring` | `std_msgs/Bool` | `hybrid_algorithm_node` | `sensor_node` |
| `/cmd_vel` | `geometry_msgs/Twist` | Algorithm node | Gazebo (bridge) |
| `/path_trace` | `nav_msgs/Path` | Algorithm node | RViz |
| `/performance_log` | `std_msgs/String` | Algorithm node | `logger_node` |
| `/odom` | `nav_msgs/Odometry` | Gazebo (bridge) | Algorithm node |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo (bridge) | All nodes |

---

## 👥 Team

| Member | Responsibility |
|--------|---------------|
| Member 1 | Robot URDF + all 3 Gazebo maze world files |
| Member 2 | `sensor_node.py` + noise parameter + `/robot_state` topic |
| Member 3 | `hybrid_algorithm_node.py` + `bug2_baseline_node.py` |
| Member 4 | RViz config + `logger_node.py` + matplotlib graphs + all launch files |

---

## 📄 License

This project was developed as part of a university robotics course.

*Platform: ROS2 Jazzy | Gazebo Harmonic | Ubuntu 24.04*


Terminal 1: Simulation & Controller
bash
cd ~/Documents/MAR_project
source install/setup.bash
ros2 launch maze_robot complex_maze_hybrid.launch.py


Terminal 2: SLAM Mapping
bash
cd ~/Documents/MAR_project
source install/setup.bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true


Terminal 3: Frontier Exploration
bash
cd ~/Documents/MAR_project
source install/setup.bash
ros2 run maze_robot frontier_node
Now watch RViz. The robot should pivot cleanly when it meets a wall and keep exploring!