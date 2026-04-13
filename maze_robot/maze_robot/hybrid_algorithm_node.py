#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

class HybridNode(Node):
    def __init__(self):
        super().__init__('hybrid_node')

        # Parameters
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 4.0)
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.start_x = None
        self.start_y = None

        # Sensor data
        self.scan_front = 10.0
        self.scan_left = 10.0

        # Modes
        self.mode = "NAV"

        # Bug2 variables
        self.hit_point_dist = float('inf')
        self.m_line_len = 1.0

        # Stability helpers
        self.blocked_counter = 0
        self.stuck_counter = 0
        self.prev_dist = None

        self.create_timer(0.05, self.control_loop)

    def euler_from_quaternion(self, q):
        return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = self.euler_from_quaternion(msg.pose.pose.orientation)

        # Set start point once
        if self.start_x is None:
            self.start_x = self.x
            self.start_y = self.y
            self.m_line_len = math.hypot(
                self.goal_x - self.start_x,
                self.goal_y - self.start_y
            )

    def scan_callback(self, msg):
        def avg(r):
            r = [x for x in r if not math.isinf(x)]
            return sum(r)/len(r) if r else 10.0

        self.scan_front = avg(list(msg.ranges[0:5]) + list(msg.ranges[355:360]))
        self.scan_left = avg(msg.ranges[85:95])

    def distance_to_goal(self):
        return math.hypot(self.goal_x - self.x, self.goal_y - self.y)

    def distance_to_m_line(self):
        if self.m_line_len == 0:
            return 0.0
        return abs(
            (self.goal_y - self.start_y)*(self.x - self.start_x) -
            (self.goal_x - self.start_x)*(self.y - self.start_y)
        ) / self.m_line_len

    def control_loop(self):
        twist = Twist()

        # Goal reached
        if self.distance_to_goal() < 0.3:
            self.cmd_pub.publish(Twist())
            self.get_logger().info("GOAL REACHED")
            return

        # 🔥 MODE SWITCHING
        if self.scan_front < 0.4:
            self.blocked_counter += 1
        else:
            self.blocked_counter = 0

        if self.blocked_counter > 5:
            if self.mode != "BUG2":
                self.mode = "BUG2"
                self.hit_point_dist = self.distance_to_goal()

        # 🔁 EXIT BUG2
        if self.mode == "BUG2":
            if (self.distance_to_m_line() < 0.25 and
                self.distance_to_goal() < self.hit_point_dist - 0.3):
                self.mode = "NAV"
                return

            # Escape condition
            if self.distance_to_goal() < self.hit_point_dist - 0.5:
                self.mode = "NAV"
                return

        # 🟢 NAV MODE
        if self.mode == "NAV":
            angle = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
            diff = angle - self.yaw

            while diff > math.pi:
                diff -= 2 * math.pi
            while diff < -math.pi:
                diff += 2 * math.pi

            twist.angular.z = 0.5 * diff
            twist.linear.x = 0.25

        # 🔵 BUG2 MODE
        elif self.mode == "BUG2":
            if self.scan_front < 0.4:
                twist.angular.z = -0.8
                twist.linear.x = 0.0
            else:
                if self.scan_left < 0.3:
                    twist.linear.x = 0.15
                    twist.angular.z = -0.3
                elif self.scan_left > 0.5:
                    twist.linear.x = 0.15
                    twist.angular.z = 0.4
                else:
                    twist.linear.x = 0.15

        # 🔴 STUCK DETECTION
        if self.prev_dist is not None:
            if abs(self.distance_to_goal() - self.prev_dist) < 0.005:
                self.stuck_counter += 1
            else:
                self.stuck_counter = 0

        if self.stuck_counter > 20:
            twist.angular.z = 0.8
            twist.linear.x = 0.0

        self.prev_dist = self.distance_to_goal()

        # 🔒 SAFETY CLAMP
        twist.linear.x = max(min(twist.linear.x, 0.3), -0.3)
        twist.angular.z = max(min(twist.angular.z, 1.0), -1.0)

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = HybridNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()