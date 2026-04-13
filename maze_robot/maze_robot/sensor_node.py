#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
import math
import random

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')
        
        # Declare noise parameter
        self.declare_parameter('noise_std_dev', 0.02)
        
        # Publishers and Subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.state_pub = self.create_publisher(String, '/robot_state', 10)
        
        # Internal topic subscription to resume exploring (from Algorithm Node)
        self.resume_sub = self.create_subscription(Bool, '/resume_exploring', self.resume_callback, 10)
        
        self.current_state = "EXPLORING"
        self.wall_threshold = 0.4  # Starts wall following if distance < 0.4
        
        # Publish state continuously at 10Hz
        self.timer = self.create_timer(0.1, self.publish_state)
        self.get_logger().info("Sensor node started.")

    def resume_callback(self, msg):
        if msg.data and self.current_state == "WALL_FOLLOWING":
            self.current_state = "EXPLORING"
            self.get_logger().info("Sensor Node: Resuming EXPLORING mode.")

    def scan_callback(self, msg):
        noise_std_dev = self.get_parameter('noise_std_dev').value
        
        # Process the scan and add gaussian noise
        noisy_ranges = []
        for r in msg.ranges:
            if math.isinf(r) or math.isnan(r):
                noisy_ranges.append(10.0) # Assume max range
            else:
                noise = random.gauss(0.0, noise_std_dev)
                noisy_ranges.append(r + noise)
        
        # Our URDF creates 360 samples from 0 to 2*pi
        # index 0 is front. Let's look at the front cone (-5 to +5 degrees)
        # Because index 355-359 are slightly right, and 0-5 are slightly left.
        front_ranges = noisy_ranges[0:5] + noisy_ranges[355:360]
        min_front_dist = min(front_ranges)
        
        if self.current_state == "EXPLORING" and min_front_dist < self.wall_threshold:
            self.current_state = "WALL_FOLLOWING"
            self.get_logger().info(f"Sensor Node: Wall detected at {min_front_dist:.2f}m. Switching to WALL_FOLLOWING.")

    def publish_state(self):
        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
