#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import random


class HybridNode(Node):
    def __init__(self):
        super().__init__('hybrid_node')

        # Goal received from frontier_node via /goal_pose
        # Start at current position so robot waits for first frontier goal
        self.goal_x = 0.0
        self.goal_y = 0.0
        self._goal_received = False

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.scan_front = 10.0
        self.scan_left = 10.0
        self.scan_right = 10.0

        self.mode = "NAV"

        self.blocked_counter = 0
        self.stuck_counter = 0
        self.prev_x = None
        self.prev_y = None

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
        self.scan_right = avg(msg.ranges[265:275])

    def distance_to_goal(self):
        return math.hypot(self.goal_x - self.x, self.goal_y - self.y)

    # 🗺️ Callback: goal from frontier_node
    def goal_callback(self, msg):
        # If we recently chose a fallback goal, ignore frontier temporarily 
        # so we actually have time to escape
        if getattr(self, 'ignore_frontier_cycles', 0) > 0:
            return

        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self._goal_received = True
        self.get_logger().debug(
            f'[FRONTIER] New goal received → x={self.goal_x:.2f}, y={self.goal_y:.2f}'
        )

    # 🔥 Fallback: random escape if stuck
    def choose_new_goal(self):
        angle = random.uniform(-math.pi, math.pi)
        dist = 2.0
        self.goal_x = self.x + dist * math.cos(angle)
        self.goal_y = self.y + dist * math.sin(angle)
        self.ignore_frontier_cycles = 100  # ignore frontier for ~5 seconds (at 20Hz)
        self.get_logger().info(f'[FALLBACK] Stuck! Random escape goal: {self.goal_x:.2f}, {self.goal_y:.2f}')

    def control_loop(self):
        twist = Twist()

        if getattr(self, 'ignore_frontier_cycles', 0) > 0:
            self.ignore_frontier_cycles -= 1

        # Wait for first frontier goal before moving
        if not self._goal_received:
            self.cmd_pub.publish(twist)  # publish zero velocity
            return

        # GOAL REACHED — stop moving and wait for new goal
        if self.distance_to_goal() < 0.4:
            self.cmd_pub.publish(twist)  # STOP completely!
            return

        # OBSTACLE DETECTION
        if self.scan_front < 0.35:
            self.blocked_counter += 1
        else:
            self.blocked_counter = 0

        # STUCK DETECTION (movement-based)
        if self.prev_x is not None:
            if math.hypot(self.x - self.prev_x, self.y - self.prev_y) < 0.01:
                self.stuck_counter += 1
            else:
                self.stuck_counter = 0

        self.prev_x = self.x
        self.prev_y = self.y

        # 🔥 IF STUCK → NEW FRONTIER GOAL
        if self.stuck_counter > 40:
            self.choose_new_goal()
            self.stuck_counter = 0

        # NAVIGATION
        angle = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
        diff = angle - self.yaw

        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi

        # 🔵 BUG2-LIKE OBSTACLE AVOIDANCE
        if self.scan_front < 0.4:
            # ONLY pivot! Do NOT move forward here, otherwise we get wedged into the wall
            twist.angular.z = 0.8
            twist.linear.x = 0.0

        else:
            # steer based on free space
            if self.scan_left > self.scan_right:
                twist.angular.z = 0.3
            else:
                twist.angular.z = -0.3

            twist.angular.z += 0.5 * diff
            twist.linear.x = 0.2

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = HybridNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()