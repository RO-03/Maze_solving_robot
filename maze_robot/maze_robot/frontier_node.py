#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np
import random


class FrontierNode(Node):
    def __init__(self):
        super().__init__('frontier_node')

        # Subscribe to SLAM map
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)

        # Publish exploration goals to the hybrid controller
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.map = None

        # Only send a new frontier every N map callbacks to avoid flooding
        self._callback_count = 0
        self._publish_every = 5  # publish a new goal every 5 map updates

        self.get_logger().info('FrontierNode started — waiting for /map from SLAM...')

    def map_callback(self, msg):
        self.map = msg
        self._callback_count += 1
        if self._callback_count >= self._publish_every:
            self._callback_count = 0
            self.find_frontier()

    def find_frontier(self):
        data = np.array(self.map.data, dtype=np.int8)
        width = self.map.info.width
        height = self.map.info.height

        if width == 0 or height == 0:
            return

        grid = data.reshape((height, width))

        frontiers = []

        # A frontier cell is an UNKNOWN cell (-1) that is adjacent to a FREE cell (0)
        for i in range(1, height - 1):
            for j in range(1, width - 1):
                if grid[i, j] == -1:          # unknown
                    neighbors = grid[i-1:i+2, j-1:j+2]
                    if 0 in neighbors:         # adjacent to free space
                        frontiers.append((i, j))

        if not frontiers:
            self.get_logger().info('No frontiers found — map may be fully explored.')
            return

        # Pick a random frontier from the candidates
        fi, fj = random.choice(frontiers)

        resolution = self.map.info.resolution
        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y

        goal_x = origin_x + fj * resolution
        goal_y = origin_y + fi * resolution

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'map'
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)
        self.get_logger().info(
            f'Frontier goal published → x={goal_x:.2f}, y={goal_y:.2f} '
            f'({len(frontiers)} frontiers found)'
        )


def main(args=None):
    rclpy.init(args=args)
    node = FrontierNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
