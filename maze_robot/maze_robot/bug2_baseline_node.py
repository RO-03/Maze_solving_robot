#!/usr/bin/env python3
"""
bug2_baseline_node.py  –  Production-grade Pure Bug2 Baseline Controller
=========================================================================
This node is the Bug2 *baseline* used for performance comparison.
It differs from hybrid_algorithm_node in that it never leaves the wall
to re-join the M-line – it simply keeps following the wall until the
goal is in view on the M-line (or times out).

Improvements over the MVP:
  1. TF2-based pose (map frame) instead of raw /odom.
  2. Stuck detection & recovery (same as hybrid node).
  3. Hysteresis-free clean state reads from sensor_node.
  4. Proper angle normalisation.
  5. Shared configurable parameters.
"""

import math
import random
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


def normalise_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class Bug2BaselineNode(Node):
    def __init__(self):
        super().__init__('bug2_baseline_node')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('goal_x',            0.0)
        self.declare_parameter('goal_y',            4.0)
        self.declare_parameter('map_frame',         'map')
        self.declare_parameter('robot_frame',       'base_footprint')
        self.declare_parameter('use_tf',            True)
        self.declare_parameter('max_linear_speed',  0.28)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('kp_angular',        1.2)
        self.declare_parameter('goal_radius',       0.4)
        self.declare_parameter('front_stop_dist',   0.35)
        self.declare_parameter('front_wedge_dist',  0.22)
        self.declare_parameter('side_target_dist',  0.35)
        self.declare_parameter('side_tol',          0.08)
        self.declare_parameter('stuck_timeout',     6.0)
        self.declare_parameter('stuck_move_tol',    0.05)

        self.goal_x      = self.get_parameter('goal_x').value
        self.goal_y      = self.get_parameter('goal_y').value
        self.map_frame   = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.use_tf      = self.get_parameter('use_tf').value

        self.max_lin     = self.get_parameter('max_linear_speed').value
        self.max_ang     = self.get_parameter('max_angular_speed').value
        self.kp          = self.get_parameter('kp_angular').value
        self.goal_radius = self.get_parameter('goal_radius').value
        self.front_stop  = self.get_parameter('front_stop_dist').value
        self.front_wedge = self.get_parameter('front_wedge_dist').value
        self.side_target = self.get_parameter('side_target_dist').value
        self.side_tol    = self.get_parameter('side_tol').value
        self.stuck_to    = self.get_parameter('stuck_timeout').value
        self.stuck_tol   = self.get_parameter('stuck_move_tol').value

        # ── TF2 ────────────────────────────────────────────────────────────
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── Pose ──────────────────────────────────────────────────────────
        self.x = self.y = self.yaw = 0.0

        # ── LIDAR ─────────────────────────────────────────────────────────
        self.scan_front = self.scan_left = self.scan_right = 10.0

        # ── State ─────────────────────────────────────────────────────────
        self.robot_state      = 'EXPLORING'
        self.goal_reached     = False
        self.collision_cooldown = False

        # ── Stuck ─────────────────────────────────────────────────────────
        self.last_moved_time  = self.get_clock().now()
        self.stuck_ref_x      = 0.0
        self.stuck_ref_y      = 0.0
        self.in_recovery      = False
        self.recovery_end_time = None

        # ── Performance ───────────────────────────────────────────────────
        self.start_time        = self.get_clock().now()
        self.total_path_length = 0.0
        self.prev_x = self.prev_y = None
        self.wall_collisions   = 0
        self.mode_switches     = 0

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.map_frame

        # ── Pub / Sub ─────────────────────────────────────────────────────
        self.create_subscription(Odometry,  '/odom',        self.odom_callback,  10)
        self.create_subscription(LaserScan, '/scan',        self.scan_callback,  10)
        self.create_subscription(String,    '/robot_state', self.state_callback, 10)

        self.cmd_pub  = self.create_publisher(Twist,  '/cmd_vel',         10)
        self.path_pub = self.create_publisher(Path,   '/path_trace',      10)
        self.log_pub  = self.create_publisher(String, '/performance_log', 10)

        self.create_timer(0.05, self.control_loop)
        self.get_logger().info(
            f'Bug2BaselineNode | goal=({self.goal_x}, {self.goal_y}) | use_tf={self.use_tf}'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────

    def odom_callback(self, msg: Odometry):
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y

        if not self.use_tf:
            self.x   = ox
            self.y   = oy
            q        = msg.pose.pose.orientation
            self.yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )

        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = 'odom'
        pose.pose            = msg.pose.pose
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)

        if self.prev_x is not None:
            self.total_path_length += math.hypot(ox - self.prev_x, oy - self.prev_y)
        self.prev_x = ox
        self.prev_y = oy

    def scan_callback(self, msg: LaserScan):
        def safe_avg(rays):
            valid = [r for r in rays
                     if not (math.isinf(r) or math.isnan(r)) and r > 0.0]
            return sum(valid) / len(valid) if valid else 10.0

        n = len(msg.ranges)
        front_idx   = list(range(0, 6)) + list(range(n - 5, n))
        self.scan_front = safe_avg([msg.ranges[i] for i in front_idx])
        self.scan_left  = safe_avg(list(msg.ranges[n // 4 - 5: n // 4 + 5]))

        if self.scan_front < 0.15:
            if not self.collision_cooldown:
                self.wall_collisions += 1
                self.collision_cooldown = True
        elif self.scan_front > 0.25:
            self.collision_cooldown = False

    def state_callback(self, msg: String):
        new_state = msg.data
        if new_state != self.robot_state:
            self.mode_switches += 1
            self.robot_state = new_state

    # ── TF ────────────────────────────────────────────────────────────────

    def _update_pose_from_tf(self) -> bool:
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
            self.x   = t.transform.translation.x
            self.y   = t.transform.translation.y
            q        = t.transform.rotation
            self.yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            return True
        except (LookupException, ConnectivityException, ExtrapolationException):
            return False

    # ── Stuck ─────────────────────────────────────────────────────────────

    def _check_and_recover_stuck(self) -> bool:
        moved = math.hypot(self.x - self.stuck_ref_x, self.y - self.stuck_ref_y)
        now   = self.get_clock().now()

        if moved > self.stuck_tol:
            self.stuck_ref_x     = self.x
            self.stuck_ref_y     = self.y
            self.last_moved_time = now
            return False

        elapsed = (now - self.last_moved_time).nanoseconds / 1e9

        if self.in_recovery:
            if self.recovery_end_time and now >= self.recovery_end_time:
                self.in_recovery       = False
                self.recovery_end_time = None
                self.cmd_pub.publish(Twist())
                return False
            # Continue recovery
            twist = Twist()
            elapsed_recovery = (self.recovery_end_time - now).nanoseconds / 1e9
            if elapsed_recovery > 2.0:
                twist.linear.x = -0.15
            else:
                twist.angular.z = 0.9
            self.cmd_pub.publish(twist)
            return True

        if elapsed > self.stuck_to:
            self.get_logger().warn('Stuck! Executing recovery.')
            self.in_recovery       = True
            self.recovery_end_time = now + Duration(seconds=3.0)
            self.last_moved_time   = now
            return True

        return False

    def _stop(self):
        self.cmd_pub.publish(Twist())

    # ── Control loop ──────────────────────────────────────────────────────

    def control_loop(self):
        if self.goal_reached:
            return

        if self.use_tf:
            self._update_pose_from_tf()

        if self._check_and_recover_stuck():
            return

        dist_goal = math.hypot(self.goal_x - self.x, self.goal_y - self.y)

        if dist_goal < self.goal_radius:
            self._stop()
            self.goal_reached = True
            time_taken = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            log_str = (
                f'BASELINE,{time_taken:.2f},{self.total_path_length:.2f},'
                f'{self.wall_collisions},{self.mode_switches}'
            )
            s_msg = String()
            s_msg.data = log_str
            self.log_pub.publish(s_msg)
            self.get_logger().info(f'Goal reached! Time={time_taken:.1f}s')
            return

        twist = Twist()

        if self.robot_state == 'EXPLORING':
            angle_to_goal = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
            angle_diff    = normalise_angle(angle_to_goal - self.yaw)

            if self.scan_front < self.front_wedge:
                twist.linear.x  = 0.0
                twist.angular.z = -self.max_ang
            elif self.scan_front < self.front_stop:
                twist.linear.x  = 0.0
                twist.angular.z = self.kp * angle_diff
            elif abs(angle_diff) > 0.25:
                twist.linear.x  = 0.05
                twist.angular.z = self.kp * angle_diff
            else:
                twist.linear.x  = self.max_lin
                twist.angular.z = 0.4 * self.kp * angle_diff

        elif self.robot_state == 'WALL_FOLLOWING':
            # Pure wall follower – no M-line exit (that's what makes it "Baseline Bug2")
            if self.scan_front < self.front_stop:
                twist.linear.x  = 0.0 if self.scan_front < self.front_wedge else 0.05
                twist.angular.z = -0.6
            else:
                side_err = self.scan_left - self.side_target
                if abs(side_err) < self.side_tol:
                    twist.linear.x  = self.max_lin * 0.85
                    twist.angular.z = 0.0
                elif side_err < 0:
                    twist.linear.x  = self.max_lin * 0.7
                    twist.angular.z = -0.4
                else:
                    twist.linear.x  = self.max_lin * 0.6
                    twist.angular.z = 0.5

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = Bug2BaselineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
