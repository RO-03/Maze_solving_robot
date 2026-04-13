#!/usr/bin/env python3
"""
Self-contained pure Bug2 Baseline (left-hand wall follower, no M-line shortcut).
State managed entirely inside this node.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math


class Bug2BaselineNode(Node):

    def __init__(self):
        super().__init__('bug2_baseline_node')

        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 4.0)
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        self.get_logger().info(
            f"Bug2 Baseline Node → Goal: ({self.goal_x}, {self.goal_y})")

        self.create_subscription(Odometry,  '/odom', self._odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

        self.cmd_pub   = self.create_publisher(Twist,  '/cmd_vel',         10)
        self.path_pub  = self.create_publisher(Path,   '/path_trace',      10)
        self.log_pub   = self.create_publisher(String, '/performance_log', 10)
        self.state_pub = self.create_publisher(String, '/robot_state',     10)

        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0

        # ── Laser readings (m) ─────────────────────────────────────────
        self.front     = 10.0
        self.left_min  = 10.0

        self.OBS_THRESH  = 0.45
        self.TARGET_DIST = 0.35
        self.state       = 'EXPLORING'

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

        self.create_timer(0.05, self._loop)

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
            self.total_path_length += math.hypot(
                self.x - self.prev_x, self.y - self.prev_y)
        self.prev_x, self.prev_y = self.x, self.y

    def _scan_cb(self, msg):
        n = len(msg.ranges)

        def _min(indices):
            vals = [msg.ranges[i] for i in indices
                    if 0 <= i < n and math.isfinite(msg.ranges[i])]
            return min(vals) if vals else 10.0

        self.front = _min(list(range(0, 20)) + list(range(n-20, n)))
        self.left_min = _min(range(20, 160))

        if self.front < 0.15 and not self.coll_cooldown:
            self.wall_collisions += 1
            self.coll_cooldown = True
        elif self.front > 0.25:
            self.coll_cooldown = False

    def _norm(self, a):
        while a >  math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

    def _dist_goal(self):
        return math.hypot(self.goal_x - self.x, self.goal_y - self.y)

    def _loop(self):
        if self.goal_reached:
            return

        if self._dist_goal() < 0.30:
            self.cmd_pub.publish(Twist())
            self.goal_reached = True
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            log = (f"BASELINE,{elapsed:.2f},"
                   f"{self.total_path_length:.2f},"
                   f"{self.wall_collisions},"
                   f"{self.mode_switches}")
            sm = String(); sm.data = log
            self.log_pub.publish(sm)
            self.get_logger().info(
                f"GOAL REACHED! Time={elapsed:.1f}s  Path={self.total_path_length:.1f}m")
            return

        twist = Twist()

        if self.state == 'EXPLORING':
            if self.front < self.OBS_THRESH:
                self.mode_switches += 1
                self.state = 'WALL_FOLLOWING'
                sm = String(); sm.data = 'WALL_FOLLOWING'
                self.state_pub.publish(sm)
                self.get_logger().info("WALL HIT → WALL_FOLLOWING")
                twist.linear.x  = 0.0
                twist.angular.z = -0.6
            else:
                angle_to_goal = math.atan2(
                    self.goal_y - self.y, self.goal_x - self.x)
                err = self._norm(angle_to_goal - self.yaw)
                if abs(err) > 0.15:
                    twist.linear.x  = 0.05
                    twist.angular.z = 0.7 * err
                else:
                    twist.linear.x  = 0.28
                    twist.angular.z = 0.1 * err

        else:  # WALL_FOLLOWING — pure left-hand rule, never exits
            if self.front < self.OBS_THRESH:
                twist.linear.x  = 0.0
                twist.angular.z = -0.8
            elif self.left_min > 0.80:
                twist.linear.x  = 0.15
                twist.angular.z = 0.65
            else:
                err = self.left_min - self.TARGET_DIST
                Kp = 2.0
                w = Kp * err
                w = max(-0.6, min(0.6, w))
                twist.linear.x  = 0.25
                twist.angular.z = w

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = Bug2BaselineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
