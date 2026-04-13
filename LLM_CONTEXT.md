# Project Context & Architecture

**Target Audience:** Any LLM or AI Assistant working on this codebase.

## 1. Project Objective
The overarching goal of this project is to build an autonomous maze-solving robot using **ROS 2 (Jazzy)** and **Gazebo Harmonic**. The robot must be able to spawn in an unknown maze, dynamically map the environment using LIDAR (SLAM), identify unexplored areas (frontiers), and navigate to them safely while avoiding obstacles. 

## 2. System Architecture
The current setup relies on a multi-node architecture:

*   **Sensors & Simulation:** 
    *   Gazebo Simulates the environment (`maze_complex.world`).
    *   `ros_gz_bridge` passes `/scan` (LIDAR) and `/odom` to ROS 2.
*   **SLAM System (`slam_toolbox`):**
    *   Subscribes to `/scan` and `/tf`.
    *   Publishes the live occupancy grid to `/map`.
    *   Maintains the `map -> odom` TF tree.
*   **Frontier Exploration Node (`frontier_node.py`):**
    *   Subscribes to `/map`.
    *   Scans the Occupancy Grid to find "Frontiers" (cells where `UNKNOWN (-1)` touches `FREE (0)` space).
    *   Publishes a dynamic `PoseStamped` goal to `/goal_pose`.
*   **Robot Controller (`hybrid_algorithm_node.py`):**
    *   Subscribes to `/odom` (for current position), `/scan` (for obstacle avoidance), and `/goal_pose` (for destination).
    *   Publishes `Twist` commands to `/cmd_vel`.
    *   Implements a hybrid state machine: moving towards the goal using proportional angular correction, and dropping into a "Bug2-like" obstacle avoidance scheme (pivoting in place) if `scan_front` detects a wall.

## 3. What We Have Done So Far
*   **Integrated `slam_toolbox`:** Successfully set up asynchronous online SLAM to build a map live using LIDAR data.
*   **Built `frontier_node.py`:** Created a custom Python script that converts the 1D `/map` data array into a 2D numpy array, searches for frontier edges, and publishes a randomly selected frontier coordinate to the controller.
*   **Refactored `hybrid_algorithm_node.py`:**
    *   Removed the static hardcoded goal and wired it to accept dynamic goals via `/goal_pose`.
    *   **Fixed Wall Wedging Bug:** Prevented the robot from wedging into corners by forcing `linear.x = 0.0` when pivoting away from a close wall (`scan_front < 0.4`).
    *   **Fixed Goal Arrival Bug:** Made the robot publish a `Twist(0.0, 0.0)` stop command and wait when it reaches within `0.4m` of its goal, allowing `frontier_node` to pick a new destination safely.
    *   **Added Stuck Fallback Cooldown:** If the robot is completely stuck, it picks a short-distance random fallback goal to escape. Added a cooldown (`ignore_frontier_cycles`) to prevent the frontier node from instantly overwriting the escape attempt.

## 4. Current Limitations & What We Must Improve Next

The architecture is functioning, but it is currently a "Minimum Viable Product" for autonomous exploration. The next AI/Developer should focus on the following core improvements:

### A. Coordinate Frame Mismatch (CRITICAL)
*   **The Bug:** `frontier_node` calculates the goal coordinates based on the `/map` origin. However, `hybrid_algorithm_node` determines its own `self.x` and `self.y` directly from the `/odom` topic.
*   **The Impact:** As the robot explores, wheel slip and SLAM loop-closures will cause `/odom` to drift relative to `/map`. Because the controller mixes coordinates from two different frames, the robot will start driving to incorrect locations if the map shifts.
*   **The Fix:** Introduce `tf2_ros` in `hybrid_algorithm_node` to look up the transform from `map` to `base_footprint` to get the robot's true map-frame position, rather than blindly trusting the raw `/odom` topic.

### B. Frontier Selection Heuristics
*   **The Bug:** `frontier_node` currently uses `random.choice(frontiers)` to pick where to go next.
*   **The Impact:** The robot might be at coordinate `(1,1)` and randomly choose a frontier at `(10,10)`, skipping unexplored frontiers right next to it. This leads to massive back-tracking and incredibly inefficient exploration.
*   **The Fix:** Implement a "Nearest Frontier" heuristic in `frontier_node.py`. Calculate the Euclidean distance from the robot to all candidate frontiers and select the closest one.

### C. Advanced Local Navigation / Path Planning
*   **The Bug:** The controller just calculates the straight-line angle to the goal and tries to drive towards it. If an obstacle is in the way, it pivots. 
*   **The Impact:** It relies entirely on line-of-sight navigation and a reactive Bug algorithm. It will struggle heavily with U-shaped traps and complex mazes because it has no global path planning (like A* or Dijkstra's).
*   **The Fix:** Either implement a proper `Nav2` (Navigation 2) stack to handle costmaps and path planning, OR implement A* search inside the python nodes using the discovered `/map`.

### D. Frontier Node Efficiency
*   **The Impact:** The frontier detection uses nested Python `for` loops scanning over a potentially massive 2D array. As the map grows large, this will cause significant CPU lag.
*   **The Fix:** Vectorize the frontier search using pure `numpy` convolution filters/masks instead of `for i... for j...` loops.
