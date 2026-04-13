#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math

class Bug2Node(Node):
    def __init__(self):
        super().__init__('bug2_node')

        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 4.0)
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.scan_front = 10.0
        self.scan_left = 10.0

        self.state = "GO_TO_GOAL"
        self.hit_point_dist = float('inf')

        self.m_line_len = math.hypot(self.goal_x, self.goal_y)

        self.create_timer(0.05, self.control_loop)

    def euler_from_quaternion(self, q):
        return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = self.euler_from_quaternion(msg.pose.pose.orientation)

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
        return abs(self.goal_x*self.y - self.goal_y*self.x)/self.m_line_len

    def control_loop(self):
        twist = Twist()

        if self.distance_to_goal() < 0.3:
            self.cmd_pub.publish(Twist())
            self.get_logger().info("GOAL REACHED")
            return

        if self.state == "GO_TO_GOAL":
            if self.scan_front < 0.4:
                self.state = "FOLLOW_WALL"
                self.hit_point_dist = self.distance_to_goal()
                return

            angle = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
            diff = angle - self.yaw

            while diff > math.pi: diff -= 2*math.pi
            while diff < -math.pi: diff += 2*math.pi

            twist.angular.z = 0.5 * diff
            twist.linear.x = 0.2

        elif self.state == "FOLLOW_WALL":
            # Leave condition (TRUE BUG2)
            if (self.distance_to_m_line() < 0.1 and
                self.distance_to_goal() < self.hit_point_dist - 0.2):
                self.state = "GO_TO_GOAL"
                return

            # Wall following (left-hand rule)
            if self.scan_front < 0.4:
                twist.angular.z = -0.6
            else:
                if self.scan_left < 0.3:
                    twist.linear.x = 0.15
                    twist.angular.z = -0.3
                elif self.scan_left > 0.5:
                    twist.linear.x = 0.15
                    twist.angular.z = 0.4
                else:
                    twist.linear.x = 0.2

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = Bug2Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()