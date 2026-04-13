#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
import math
import time

class HybridAlgorithmNode(Node):
    def __init__(self):
        super().__init__('hybrid_algorithm_node')
        
        # Parameters
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 4.0)
        
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value
        
        self.get_logger().info(f"Hybrid Node targeting Goal: ({self.goal_x}, {self.goal_y})")
        
        # Subs and Pubs
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(String, '/robot_state', self.state_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/path_trace', 10)
        self.log_pub = self.create_publisher(String, '/performance_log', 10)
        self.resume_pub = self.create_publisher(Bool, '/resume_exploring', 10)
        
        # State variables
        self.robot_state = "EXPLORING"
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        self.scan_front = 10.0
        self.scan_left = 10.0
        self.scan_right = 10.0
        
        # Math & Tracking
        self.hit_point_dist = float('inf')
        self.m_line_threshold = 0.1 # distance to M-line to be considered ON it
        
        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom"
        
        # Performance tracking
        self.start_time = self.get_clock().now()
        self.total_path_length = 0.0
        self.prev_x = None
        self.prev_y = None
        self.wall_collisions = 0
        self.mode_switches = 0
        self.goal_reached = False
        self.collision_cooldown = False
        
        # Control Timer
        self.create_timer(0.05, self.control_loop)

        # M-Line equation components (line from origin to goal)
        self.m_line_len = math.hypot(self.goal_x, self.goal_y)

    def euler_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        
        # Update Path
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "odom"
        pose.pose = msg.pose.pose
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)
        
        # Update Distance traversed
        if self.prev_x is not None:
            self.total_path_length += math.hypot(self.x - self.prev_x, self.y - self.prev_y)
        self.prev_x = self.x
        self.prev_y = self.y

    def scan_callback(self, msg):
        # Ray 0 is front, 90 is left, 270 is right in our setup
        # Average a few rays to avoid noise spikes causing erratic turning
        def avg(rays):
            valid = [r for r in rays if not math.isinf(r) and not math.isnan(r)]
            return sum(valid)/len(valid) if valid else 10.0
            
        self.scan_front = avg(list(msg.ranges[0:5]) + list(msg.ranges[355:360]))
        self.scan_left = avg(msg.ranges[85:95])
        self.scan_right = avg(msg.ranges[265:275])
        
        # Collision counting (front < 0.15m)
        if self.scan_front < 0.15:
            if not self.collision_cooldown:
                self.wall_collisions += 1
                self.collision_cooldown = True
        elif self.scan_front > 0.2:
            self.collision_cooldown = False

    def state_callback(self, msg):
        new_state = msg.data
        if new_state != self.robot_state:
            # We switched states
            self.mode_switches += 1
            if new_state == "WALL_FOLLOWING":
                self.hit_point_dist = self.distance_to_goal()
            self.robot_state = new_state

    def distance_to_goal(self):
        return math.hypot(self.goal_x - self.x, self.goal_y - self.y)

    def distance_to_m_line(self):
        # Distance from point (x,y) to line joining (0,0) and (goal_x, goal_y)
        if self.m_line_len == 0: return 0.0
        return abs(self.goal_x * self.y - self.goal_y * self.x) / self.m_line_len

    def control_loop(self):
        if self.goal_reached: return

        dist_goal = self.distance_to_goal()
        if dist_goal < 0.3:
            self.get_logger().info("GOAL REACHED!")
            TwistMsg = Twist()
            self.cmd_pub.publish(TwistMsg)
            self.goal_reached = True
            
            # Publish performance log
            time_taken = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            log_str = f"HYBRID,{time_taken:.2f},{self.total_path_length:.2f},{self.wall_collisions},{self.mode_switches}"
            s_msg = String()
            s_msg.data = log_str
            self.log_pub.publish(s_msg)
            return

        twist = Twist()

        if self.robot_state == "EXPLORING":
            # Frontier Behavior: point to goal and go straight
            inc_x = self.goal_x - self.x
            inc_y = self.goal_y - self.y
            angle_to_goal = math.atan2(inc_y, inc_x)
            
            angle_diff = angle_to_goal - self.yaw
            # Normalize angle to [-pi, pi]
            while angle_diff > math.pi: angle_diff -= 2*math.pi
            while angle_diff < -math.pi: angle_diff += 2*math.pi
            
            if abs(angle_diff) > 0.1:
                twist.angular.z = 0.5 * angle_diff
                twist.linear.x = 0.05
            else:
                twist.linear.x = 0.3
                twist.angular.z = 0.0

        elif self.robot_state == "WALL_FOLLOWING":
            # Left Hand Rule Bug2
            
            # First, check if we hit the "Leave Point" condition
            dist_m = self.distance_to_m_line()
            dist_g = self.distance_to_goal()
            
            # If we are on the M_line AND closer to goal than where we hit the wall:
            if dist_m < self.m_line_threshold and dist_g < (self.hit_point_dist - 0.2):
                msg = Bool()
                msg.data = True
                self.resume_pub.publish(msg)
                self.get_logger().info("Left Point reached! Resuming M-Line.")
                return # Skip control this cycle to let state switch
            
            # Actually follow the wall
            if self.scan_front < 0.4:
                # Obstacle in front, turn right mostly in place
                twist.linear.x = 0.0
                twist.angular.z = -0.5
            else:
                # Keep side wall around 0.35m away
                if self.scan_left < 0.25:
                    # Too close to left wall, move right slightly
                    twist.linear.x = 0.2
                    twist.angular.z = -0.3
                elif self.scan_left > 0.45:
                    # Losing left wall, move left to find it
                    if self.scan_left > 1.0:
                        twist.linear.x = 0.05
                        twist.angular.z = 0.8  # Sharp turn for corners
                    else:
                        twist.linear.x = 0.15
                        twist.angular.z = 0.5
                else:
                    # Just right, cruise forward
                    twist.linear.x = 0.25
                    twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = HybridAlgorithmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
