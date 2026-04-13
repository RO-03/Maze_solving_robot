#!/usr/bin/env python3
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
        
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(String, '/robot_state', self.state_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/path_trace', 10)
        self.log_pub = self.create_publisher(String, '/performance_log', 10)
        
        self.robot_state = "EXPLORING"
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        self.scan_front = 10.0
        self.scan_left = 10.0
        self.scan_right = 10.0
        
        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom"
        
        self.start_time = self.get_clock().now()
        self.total_path_length = 0.0
        self.prev_x = None
        self.prev_y = None
        self.wall_collisions = 0
        self.mode_switches = 0
        self.goal_reached = False
        self.collision_cooldown = False
        
        self.create_timer(0.05, self.control_loop)

    def euler_from_quaternion(self, quaternion):
        x = quaternion.x; y = quaternion.y; z = quaternion.z; w = quaternion.w
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "odom"
        pose.pose = msg.pose.pose
        self.path_msg.poses.append(pose)
        self.path_pub.publish(self.path_msg)
        
        if self.prev_x is not None:
            self.total_path_length += math.hypot(self.x - self.prev_x, self.y - self.prev_y)
        self.prev_x = self.x
        self.prev_y = self.y

    def scan_callback(self, msg):
        def avg(rays):
            valid = [r for r in rays if not math.isinf(r) and not math.isnan(r)]
            return sum(valid)/len(valid) if valid else 10.0
        self.scan_front = avg(list(msg.ranges[0:5]) + list(msg.ranges[355:360]))
        self.scan_left = avg(msg.ranges[85:95])
        
        if self.scan_front < 0.15:
            if not self.collision_cooldown:
                self.wall_collisions += 1
                self.collision_cooldown = True
        elif self.scan_front > 0.2:
            self.collision_cooldown = False

    def state_callback(self, msg):
        new_state = msg.data
        if new_state != self.robot_state:
            self.mode_switches += 1
            self.robot_state = new_state

    def control_loop(self):
        if self.goal_reached: return

        dist_goal = math.hypot(self.goal_x - self.x, self.goal_y - self.y)
        if dist_goal < 0.3:
            TwistMsg = Twist()
            self.cmd_pub.publish(TwistMsg)
            self.goal_reached = True
            time_taken = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            log_str = f"BASELINE,{time_taken:.2f},{self.total_path_length:.2f},{self.wall_collisions},{self.mode_switches}"
            s_msg = String()
            s_msg.data = log_str
            self.log_pub.publish(s_msg)
            return

        twist = Twist()

        if self.robot_state == "EXPLORING":
            inc_x = self.goal_x - self.x
            inc_y = self.goal_y - self.y
            angle_to_goal = math.atan2(inc_y, inc_x)
            
            angle_diff = angle_to_goal - self.yaw
            while angle_diff > math.pi: angle_diff -= 2*math.pi
            while angle_diff < -math.pi: angle_diff += 2*math.pi
            
            if abs(angle_diff) > 0.1:
                twist.angular.z = 0.5 * angle_diff
                twist.linear.x = 0.05
            else:
                twist.linear.x = 0.3
                twist.angular.z = 0.0

        elif self.robot_state == "WALL_FOLLOWING":
            # Pure wall follower forever (does not resume exploring via M-line)
            if self.scan_front < 0.4:
                twist.linear.x = 0.0
                twist.angular.z = -0.5
            else:
                if self.scan_left < 0.25:
                    twist.linear.x = 0.15
                    twist.angular.z = -0.3
                elif self.scan_left < 0.55:
                    twist.linear.x = 0.25
                    twist.angular.z = 0.0
                elif self.scan_left < 1.2:
                    twist.linear.x = 0.15
                    twist.angular.z = 0.4
                else:
                    # No left wall nearby — go straight
                    twist.linear.x = 0.25
                    twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = Bug2BaselineNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
