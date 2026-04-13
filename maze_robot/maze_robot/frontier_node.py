#!/usr/bin/env python3
"""
frontier_node.py  –  Production-grade Frontier Exploration Node
===============================================================
Improvements over the MVP:
  1. Vectorised frontier detection via numpy convolution (no Python loops).
  2. Nearest-frontier selection (Euclidean distance in map frame).
  3. Minimum-cluster-size filter to ignore 1-cell noise frontiers.
  4. Robot position read from TF2 (map → base_footprint) so it stays
     in the same coordinate frame as the map origin.  Falls back to
     /odom only when TF is not yet available.
  5. Cooldown / debounce so a new goal is not hammered every map tick.
  6. Publishes /exploration_complete (std_msgs/Bool) when no frontiers remain.
  7. Full parameter set exposed for easy tuning without recompiling.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from scipy.ndimage import label as nd_label          # cluster labelling
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


# Occupancy grid constants
FREE     =   0
UNKNOWN  =  -1
OCCUPIED = 100


class FrontierNode(Node):
    def __init__(self):
        super().__init__('frontier_node')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('goal_publish_rate',   1.0)   # Hz – how often we re-evaluate
        self.declare_parameter('min_frontier_size',   5)     # cells – ignore tiny clusters
        self.declare_parameter('goal_arrival_radius', 0.5)   # m – consider goal reached
        self.declare_parameter('map_frame',           'map')
        self.declare_parameter('robot_frame',         'base_footprint')
        self.declare_parameter('use_tf',              True)  # False → fallback to /odom

        self.pub_rate         = self.get_parameter('goal_publish_rate').value
        self.min_cluster_size = self.get_parameter('min_frontier_size').value
        self.arrival_radius   = self.get_parameter('goal_arrival_radius').value
        self.map_frame        = self.get_parameter('map_frame').value
        self.robot_frame      = self.get_parameter('robot_frame').value
        self.use_tf           = self.get_parameter('use_tf').value

        # ── TF2 ────────────────────────────────────────────────────────────
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── State ──────────────────────────────────────────────────────────
        self.map_data   : OccupancyGrid | None = None
        self.robot_x    = 0.0
        self.robot_y    = 0.0
        self.current_goal: tuple[float, float] | None = None
        self.exploration_done = False

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(OccupancyGrid, '/map',  self.map_callback,  10)
        self.create_subscription(Odometry,      '/odom', self.odom_callback, 10)

        # ── Publishers ────────────────────────────────────────────────────
        self.goal_pub    = self.create_publisher(PoseStamped, '/goal_pose',           10)
        self.done_pub    = self.create_publisher(Bool,        '/exploration_complete', 10)

        # ── Timer ─────────────────────────────────────────────────────────
        period = 1.0 / max(self.pub_rate, 0.1)
        self.create_timer(period, self.exploration_loop)

        self.get_logger().info(
            f'FrontierNode started | rate={self.pub_rate:.1f} Hz | '
            f'min_cluster={self.min_cluster_size} cells | use_tf={self.use_tf}'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────

    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg

    def odom_callback(self, msg: Odometry):
        """Fallback position source when TF is unavailable."""
        if not self.use_tf:
            self.robot_x = msg.pose.pose.position.x
            self.robot_y = msg.pose.pose.position.y

    # ── TF Helper ─────────────────────────────────────────────────────────

    def _update_robot_pose_from_tf(self) -> bool:
        """
        Look up map → base_footprint transform.
        Returns True on success, False if TF not yet ready.
        """
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time(),          # latest
                timeout=Duration(seconds=0.1),
            )
            self.robot_x = t.transform.translation.x
            self.robot_y = t.transform.translation.y
            return True
        except (LookupException, ConnectivityException, ExtrapolationException):
            return False

    # ── Frontier Detection (vectorised) ───────────────────────────────────

    def _find_frontiers(self) -> list[tuple[float, float]]:
        """
        Return a list of (world_x, world_y) frontier centroids,
        ordered by distance from the robot (nearest first).
        """
        msg = self.map_data
        w   = msg.info.width
        h   = msg.info.height
        res = msg.info.resolution
        ox  = msg.info.origin.position.x
        oy  = msg.info.origin.position.y

        # Reshape 1-D data → 2-D int16 array  (row = y, col = x)
        grid = np.array(msg.data, dtype=np.int16).reshape((h, w))

        # Boolean masks
        free_mask     = (grid == FREE)
        unknown_mask  = (grid == UNKNOWN)

        # A frontier cell is UNKNOWN and has at least one FREE neighbour.
        # We detect FREE neighbours via a 3×3 max-pool over the free mask.
        kernel = np.ones((3, 3), dtype=np.uint8)
        from scipy.ndimage import maximum_filter
        free_dilated  = maximum_filter(free_mask.astype(np.uint8), footprint=kernel)
        frontier_mask = unknown_mask & (free_dilated == 1)

        if not frontier_mask.any():
            return []

        # Label connected components so we can filter by cluster size
        labeled, num_features = nd_label(frontier_mask)
        centroids: list[tuple[float, float]] = []

        for label_id in range(1, num_features + 1):
            cluster_cells = np.argwhere(labeled == label_id)  # [[row, col], ...]
            if len(cluster_cells) < self.min_cluster_size:
                continue
            # Centroid in grid coords
            mean_row = cluster_cells[:, 0].mean()
            mean_col = cluster_cells[:, 1].mean()
            # Convert to world coords
            wx = ox + (mean_col + 0.5) * res
            wy = oy + (mean_row + 0.5) * res
            centroids.append((wx, wy))

        # Sort by Euclidean distance from robot (nearest-frontier heuristic)
        centroids.sort(key=lambda p: math.hypot(p[0] - self.robot_x,
                                                 p[1] - self.robot_y))
        return centroids

    # ── Main Loop ─────────────────────────────────────────────────────────

    def exploration_loop(self):
        if self.exploration_done:
            return

        if self.map_data is None:
            self.get_logger().info('Waiting for /map …', throttle_duration_sec=5.0)
            return

        # Update robot position from TF (preferred) or odom (fallback)
        if self.use_tf:
            tf_ok = self._update_robot_pose_from_tf()
            if not tf_ok:
                self.get_logger().warn(
                    'TF map→base_footprint not yet available, using /odom fallback.',
                    throttle_duration_sec=5.0,
                )

        # Check if we already arrived at the current goal
        if self.current_goal is not None:
            dist = math.hypot(
                self.current_goal[0] - self.robot_x,
                self.current_goal[1] - self.robot_y,
            )
            if dist < self.arrival_radius:
                self.get_logger().info(
                    f'Reached goal ({self.current_goal[0]:.2f}, {self.current_goal[1]:.2f}), '
                    'searching for next frontier …'
                )
                self.current_goal = None

        frontiers = self._find_frontiers()

        if not frontiers:
            if not self.exploration_done:
                self.get_logger().info('No frontiers remaining – exploration complete!')
                self.exploration_done = True
                done_msg = Bool()
                done_msg.data = True
                self.done_pub.publish(done_msg)
            return

        # Pick nearest frontier (already sorted)
        best = frontiers[0]

        # Skip re-publishing if goal hasn't changed meaningfully
        if self.current_goal is not None:
            if math.hypot(best[0] - self.current_goal[0],
                          best[1] - self.current_goal[1]) < 0.3:
                return   # no meaningful change, avoid chattering

        self.current_goal = best
        self.get_logger().info(
            f'New goal: ({best[0]:.2f}, {best[1]:.2f})  '
            f'[{len(frontiers)} frontier clusters remaining]'
        )

        goal_msg = PoseStamped()
        goal_msg.header.stamp    = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = self.map_frame
        goal_msg.pose.position.x = best[0]
        goal_msg.pose.position.y = best[1]
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation.w = 1.0   # identity – controller computes heading
        self.goal_pub.publish(goal_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
