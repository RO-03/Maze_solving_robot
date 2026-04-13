#!/usr/bin/env python3
"""
Self-contained Hybrid Bug2 + Frontier algorithm.
All state (EXPLORING / WALL_FOLLOWING) managed exclusively inside this node.

Left-hand wall follower with correct outer-corner handling:
  Priority 1 : front blocked             → turn right (CW)
  Priority 2 : no left wall (outer corner) → turn left + forward (CCW)
  Priority 3 : left wall too close       → nudge right
  Priority 4 : left wall sweet spot      → cruise straight
  Priority 5 : drifting from left wall   → nudge left

Bug2 M-line exit:
  • Traveled ≥ MIN_WF_DIST from hit-point, AND
  • perpendicular distance to M-line < M_THRESH, AND
  • closer to goal than at hit-point
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math


class HybridAlgorithmNode(Node):

    def __init__(self):
        super().__init__('hybrid_algorithm_node')

        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 4.0)
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.get_logger().info(
            f"Hybrid Bug2 Node → Goal: ({self.goal_x}, {self.goal_y})")

        # ── Topics ──────────────────────────────────────────────────────
        self.create_subscription(Odometry,  '/odom', self._odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

        self.cmd_pub   = self.create_publisher(Twist,  '/cmd_vel',         10)
        self.path_pub  = self.create_publisher(Path,   '/path_trace',      10)
        self.log_pub   = self.create_publisher(String, '/performance_log', 10)
        self.state_pub = self.create_publisher(String, '/robot_state',     10)

        # ── Robot pose ──────────────────────────────────────────────────
        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0

        # ── Laser readings (m) ─────────────────────────────────────────
        self.front     = 10.0   # front cone
        self.left_min  = 10.0   # entire left hemisphere

        # ── Bug2 state ──────────────────────────────────────────────────
        self.state            = 'EXPLORING'
        self.hit_x            = 0.0
        self.hit_y            = 0.0
        self.hit_dist_goal    = float('inf')
        self.dist_traveled_wf = 0.0

        # ── Tuning ──────────────────────────────────────────────────────
        self.OBS_THRESH  = 0.45   # front obstacle threshold
        self.TARGET_DIST = 0.35   # desired distance from left wall
        self.M_THRESH    = 0.15   # "on M-line" tolerance (m)
        self.MIN_WF_DIST = 0.80   # min wall-follow travel before exit check

        # ── Performance tracking ────────────────────────────────────────
        self.path_msg          = Path()
        self.path_msg.header.frame_id = "odom"
        self.start_time        = self.get_clock().now()
        self.total_path_length = 0.0
        self.prev_x            = None
        self.prev_y            = None
        self.wall_collisions   = 0
        self.mode_switches     = 0
        self.goal_reached      = False
        self.coll_cooldown     = False

        # M-line: straight line from (0,0) to (goal_x, goal_y)
        self.m_len = math.hypot(self.goal_x, self.goal_y)

        self.create_timer(0.05, self._loop)   # 20 Hz

    # ────────────────────────── Callbacks ───────────────────────────────

    def _yaw(self, q):
        return math.atan2(2*(q.w*q.z + q.x*q.y),
                          1.0 - 2*(q.y**2 + q.z**2))

    def _odom_cb(self, msg):
        self.x   = msg.pose.pose.position.x
        self.y   = msg.pose.pose.position.y
        self.yaw = self._yaw(msg.pose.pose.orientation)

        ps = PoseStamped()
        ps.header.stamp    = self.get_clock().now().to_msg()
        ps.header.frame_id = "odom"
        ps.pose            = msg.pose.pose
        self.path_msg.poses.append(ps)
        self.path_pub.publish(self.path_msg)

        if self.prev_x is not None:
            d = math.hypot(self.x - self.prev_x, self.y - self.prev_y)
            self.total_path_length += d
            if self.state == 'WALL_FOLLOWING':
                self.dist_traveled_wf += d
        self.prev_x, self.prev_y = self.x, self.y

    def _scan_cb(self, msg):
        n = len(msg.ranges)

        def _min(indices):
            vals = [msg.ranges[i] for i in indices
                    if 0 <= i < n and math.isfinite(msg.ranges[i])]
            return min(vals) if vals else 10.0

        # Gather specific ray indices for geometric wall tracking
        # n is 360, so 1 degree per index
        self.ray_front = _min(list(range(0, 15)) + list(range(n-15, n)))
        
        # We need two rays to compute wall angle and distance
        # Ray 1: pure left (90 deg)
        self.ray_left = msg.ranges[90] if math.isfinite(msg.ranges[90]) else 10.0
        
        # Ray 2: front-left (45 deg)
        self.ray_fl   = msg.ranges[45] if math.isfinite(msg.ranges[45]) else 10.0

        # Overall left hemisphere min (for general safety/corner detection)
        self.left_min = _min(range(15, 165))

        if self.ray_front < 0.15 and not self.coll_cooldown:
            self.wall_collisions += 1
            self.coll_cooldown = True
        elif self.ray_front > 0.25:
            self.coll_cooldown = False

    # ────────────────────────── Helpers ─────────────────────────────────

    def _dist_goal(self):
        return math.hypot(self.goal_x - self.x, self.goal_y - self.y)

    def _dist_m_line(self):
        """Perpendicular distance from (x,y) to line (0,0)→(goal_x,goal_y)."""
        if self.m_len == 0:
            return 0.0
        return abs(self.goal_x * self.y - self.goal_y * self.x) / self.m_len

    def _norm(self, a):
        while a >  math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

    def _pub_state(self, s):
        m = String(); m.data = s
        self.state_pub.publish(m)

    # ────────────────── Left-hand wall follower ──────────────────────────

    def _wall_follow(self):
        """
        Geometrically exact PD wall follower.
        Calculates wall angle (alpha) and projected distance, completely
        eliminating the sensor-dropout wobble that causes infinite loops.
        """
        twist = Twist()

        if self.ray_front < self.OBS_THRESH:
            # Obstacle in front — hard right turn in place
            twist.linear.x  = 0.0
            twist.angular.z = -0.8
            return twist

        # Geometric projection (using 45 deg and 90 deg rays)
        # alpha is angle of the wall relative to robot's heading
        if self.ray_left < 3.0 and self.ray_fl < 3.0:
            theta = math.radians(45.0)
            numerator   = self.ray_fl * math.cos(theta) - self.ray_left
            denominator = self.ray_fl * math.sin(theta)
            alpha = math.atan2(numerator, denominator)
            
            # Actual perpendicular distance to the wall
            Dt = self.ray_left * math.cos(alpha)
            
            # Predict distance slightly ahead to prevent overshoot
            lookahead = 0.3
            D_pred = Dt + lookahead * math.sin(alpha)
            
            err = self.TARGET_DIST - D_pred
            
            # PD control on predicted error
            w = -2.5 * err
            # Cap angular velocity
            w = max(-0.6, min(0.6, w))
            
            twist.linear.x  = 0.25
            twist.angular.z = w

        elif self.left_min > 0.70:
            # Wall completely lost (outer corner)
            twist.linear.x  = 0.15
            twist.angular.z = 0.65
        else:
            # We see something on the left but not the two clean rays.
            # Gentle curve left to re-acquire the rigid profile.
            twist.linear.x  = 0.20
            twist.angular.z = 0.30

        return twist

    # ────────────────────────── Control loop ─────────────────────────────

    def _loop(self):
        if self.goal_reached:
            return

        # ── Goal check ──────────────────────────────────────────────────
        if self._dist_goal() < 0.30:
            self.cmd_pub.publish(Twist())
            self.goal_reached = True
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            log = (f"HYBRID,{elapsed:.2f},"
                   f"{self.total_path_length:.2f},"
                   f"{self.wall_collisions},"
                   f"{self.mode_switches}")
            sm = String(); sm.data = log
            self.log_pub.publish(sm)
            self.get_logger().info(
                f"GOAL REACHED!  Time={elapsed:.1f}s  Path={self.total_path_length:.1f}m")
            return

        twist = Twist()

        # ── EXPLORING ───────────────────────────────────────────────────
        if self.state == 'EXPLORING':
            if self.ray_front < self.OBS_THRESH:
                # Record hit point and switch
                self.hit_x, self.hit_y = self.x, self.y
                self.hit_dist_goal     = self._dist_goal()
                self.dist_traveled_wf  = 0.0
                self.mode_switches    += 1
                self.state             = 'WALL_FOLLOWING'
                self._pub_state('WALL_FOLLOWING')
                self.get_logger().info(
                    f"WALL HIT → WALL_FOLLOWING  d_goal={self.hit_dist_goal:.2f}")
                # Start turning right immediately
                twist.linear.x  = 0.0
                twist.angular.z = -0.6
            else:
                # Steer toward goal (proportional controller)
                angle_to_goal = math.atan2(
                    self.goal_y - self.y, self.goal_x - self.x)
                err = self._norm(angle_to_goal - self.yaw)
                if abs(err) > 0.15:
                    twist.linear.x  = 0.05
                    twist.angular.z = 0.7 * err
                else:
                    twist.linear.x  = 0.28
                    twist.angular.z = 0.1 * err

        # ── WALL_FOLLOWING ──────────────────────────────────────────────
        else:
            d_m = self._dist_m_line()
            d_g = self._dist_goal()

            # Bug2 exit condition
            if (self.dist_traveled_wf > self.MIN_WF_DIST
                    and d_m < self.M_THRESH
                    and d_g < self.hit_dist_goal - 0.20):
                self.mode_switches += 1
                self.state = 'EXPLORING'
                self._pub_state('EXPLORING')
                self.get_logger().info(
                    f"M-LINE EXIT → EXPLORING  d_goal={d_g:.2f}  d_m={d_m:.3f}")
                twist.linear.x  = 0.10
                twist.angular.z = 0.0
            else:
                twist = self._wall_follow()

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = HybridAlgorithmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
