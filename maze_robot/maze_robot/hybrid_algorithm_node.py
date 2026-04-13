#!/usr/bin/env python3
"""
Smart Bug0 Heuristic Tracker for Maze Solving.
Dynamically chooses to follow the left or right wall based on "openness" of the path.
Exits wall-following instantly when a clean, clear path to the goal is detected.
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
            f"Smart Heuristic Bug0 Node → Goal: ({self.goal_x}, {self.goal_y})")

        self.create_subscription(Odometry,  '/odom', self._odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, 10)

        self.cmd_pub   = self.create_publisher(Twist,  '/cmd_vel',         10)
        self.path_pub  = self.create_publisher(Path,   '/path_trace',      10)
        self.log_pub   = self.create_publisher(String, '/performance_log', 10)
        self.state_pub = self.create_publisher(String, '/robot_state',     10)

        self.x   = 0.0
        self.y   = 0.0
        self.yaw = 0.0

        # Lidar measurements
        self.ray_front      = 10.0
        self.ray_left       = 10.0
        self.ray_fl         = 10.0
        self.ray_right      = 10.0
        self.ray_fr         = 10.0
        self.left_min       = 10.0
        self.right_min      = 10.0
        self.left_openness  = 10.0
        self.right_openness = 10.0

        self.state = 'EXPLORING'
        self.dist_traveled_wf = 0.0

        self.OBS_THRESH  = 0.45
        self.TARGET_DIST = 0.35

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
            d = math.hypot(self.x - self.prev_x, self.y - self.prev_y)
            self.total_path_length += d
            if self.state != 'EXPLORING':
                self.dist_traveled_wf += d
        self.prev_x, self.prev_y = self.x, self.y

    def _scan_cb(self, msg):
        n = len(msg.ranges)

        def _min(indices):
            vals = [msg.ranges[i] for i in indices if 0 <= i < n and math.isfinite(msg.ranges[i])]
            return min(vals) if vals else 10.0
        
        def _avg(indices):
            vals = [msg.ranges[i] for i in indices if 0 <= i < n and math.isfinite(msg.ranges[i])]
            return sum(vals)/len(vals) if vals else 10.0

        self.ray_front = _min(list(range(0, 15)) + list(range(n-15, n)))
        
        # Left hemisphere distances
        self.ray_left  = msg.ranges[90] if math.isfinite(msg.ranges[90]) else 10.0
        self.ray_fl    = msg.ranges[45] if math.isfinite(msg.ranges[45]) else 10.0
        self.left_min  = _min(range(20, 160))
        self.left_openness = _avg(range(15, 90))

        # Right hemisphere distances
        self.ray_right = msg.ranges[270] if math.isfinite(msg.ranges[270]) else 10.0
        self.ray_fr    = msg.ranges[315] if math.isfinite(msg.ranges[315]) else 10.0
        self.right_min = _min(range(200, 340))
        self.right_openness = _avg(range(270, 345))

        if self.ray_front < 0.15 and not self.coll_cooldown:
            self.wall_collisions += 1
            self.coll_cooldown = True
        elif self.ray_front > 0.25:
            self.coll_cooldown = False

    def _norm(self, a):
        while a >  math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

    def _pub_state(self, s):
        m = String(); m.data = s
        self.state_pub.publish(m)

    def _wall_follow_left(self):
        """Follow the wall on the LEFT (standard behavior)."""
        twist = Twist()
        if self.ray_front < self.OBS_THRESH:
            twist.linear.x  = 0.0
            twist.angular.z = -0.8
            return twist

        if self.ray_left < 3.0 and self.ray_fl < 3.0:
            theta = math.radians(45.0)
            alpha = math.atan2(self.ray_fl * math.cos(theta) - self.ray_left, self.ray_fl * math.sin(theta))
            D_pred = self.ray_left * math.cos(alpha) + 0.3 * math.sin(alpha)
            err = self.TARGET_DIST - D_pred
            twist.linear.x  = 0.25
            twist.angular.z = max(-0.6, min(0.6, -2.5 * err))
        elif self.left_min > 0.70:
            twist.linear.x  = 0.15
            twist.angular.z = 0.65
        else:
            twist.linear.x  = 0.20
            twist.angular.z = 0.30
        return twist

    def _wall_follow_right(self):
        """Follow the wall on the RIGHT."""
        twist = Twist()
        if self.ray_front < self.OBS_THRESH:
            # Turn left if blocked
            twist.linear.x  = 0.0
            twist.angular.z = 0.8
            return twist

        if self.ray_right < 3.0 and self.ray_fr < 3.0:
            theta = math.radians(45.0)
            alpha = math.atan2(self.ray_fr * math.cos(theta) - self.ray_right, self.ray_fr * math.sin(theta))
            D_pred = self.ray_right * math.cos(alpha) + 0.3 * math.sin(alpha)
            err = self.TARGET_DIST - D_pred  # +err means too close
            twist.linear.x  = 0.25
            twist.angular.z = max(-0.6, min(0.6, +2.5 * err)) # turn left (+) if too close
        elif self.right_min > 0.70:
            twist.linear.x  = 0.15
            twist.angular.z = -0.65
        else:
            twist.linear.x  = 0.20
            twist.angular.z = -0.30
        return twist

    def _loop(self):
        if self.goal_reached:
            return

        d_goal = math.hypot(self.goal_x - self.x, self.goal_y - self.y)
        if d_goal < 0.30:
            self.cmd_pub.publish(Twist())
            self.goal_reached = True
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            sm = String(); sm.data = f"HYBRID,{elapsed:.2f},{self.total_path_length:.2f},{self.wall_collisions},{self.mode_switches}"
            self.log_pub.publish(sm)
            self.get_logger().info(f"GOAL REACHED!  Time={elapsed:.1f}s")
            return

        twist = Twist()
        angle_to_goal = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
        err_g = self._norm(angle_to_goal - self.yaw)

        if self.state == 'EXPLORING':
            if self.ray_front < self.OBS_THRESH:
                self.mode_switches += 1
                self.dist_traveled_wf = 0.0
                # Choose wall dynamically based on free space!
                if self.right_openness > self.left_openness:
                    self.state = 'FOLLOW_LEFT_WALL'
                    self._pub_state('FOLLOW_LEFT')
                    self.get_logger().info(f"WALL HIT → FOLLOW_LEFT (R:{self.right_openness:.1f} > L:{self.left_openness:.1f})")
                    twist.linear.x = 0.0
                    twist.angular.z = -0.8
                else:
                    self.state = 'FOLLOW_RIGHT_WALL'
                    self._pub_state('FOLLOW_RIGHT')
                    self.get_logger().info(f"WALL HIT → FOLLOW_RIGHT (L:{self.left_openness:.1f} >= R:{self.right_openness:.1f})")
                    twist.linear.x = 0.0
                    twist.angular.z = 0.8
            else:
                if abs(err_g) > 0.15:
                    twist.linear.x  = 0.05
                    twist.angular.z = 0.7 * err_g
                else:
                    twist.linear.x  = 0.28
                    twist.angular.z = 0.1 * err_g

        else:
            # Smart BUG0 Exit: If we are facing the goal and the path is clear!
            if abs(err_g) < 0.35 and self.ray_front > 1.25 and self.dist_traveled_wf > 0.30:
                self.mode_switches += 1
                self.state = 'EXPLORING'
                self._pub_state('EXPLORING')
                self.get_logger().info("CLEAR SHOT TO GOAL → EXPLORING!")
                twist.linear.x = 0.10
                twist.angular.z = 0.0
            else:
                if self.state == 'FOLLOW_LEFT_WALL':
                    twist = self._wall_follow_left()
                else:
                    twist = self._wall_follow_right()

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = HybridAlgorithmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
