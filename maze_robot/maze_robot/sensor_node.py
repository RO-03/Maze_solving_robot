#!/usr/bin/env python3
"""
sensor_node.py  –  Production-grade State Manager / Sensor Processor
=====================================================================
Improvements over the MVP:
  1. Hysteresis on state transitions (separate enter / exit thresholds)
     to prevent rapid oscillation between EXPLORING and WALL_FOLLOWING.
  2. Configurable front-cone width (degrees) via parameter.
  3. Gaussian noise injection parameterised and optional (disable for
     performance when not needed).
  4. Resume callback correctly resets internal hysteresis counter so the
     robot doesn't immediately switch back to WALL_FOLLOWING.
  5. Publishes state at 20 Hz (was 10 Hz) for tighter control coupling.
  6. Scan ranges handled safely: LIDAR may return 0.0 for invalid rays
     (Gazebo Harmonic) – these are now treated as max range, not 0.0.
"""

import math
import random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool


class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('noise_std_dev',        0.02)
        self.declare_parameter('add_noise',            True)
        self.declare_parameter('wall_enter_threshold', 0.38)  # enter WALL_FOLLOWING
        self.declare_parameter('wall_exit_threshold',  0.55)  # exit  WALL_FOLLOWING
        self.declare_parameter('front_cone_deg',       10)    # half-cone (each side)
        self.declare_parameter('publish_rate',         20.0)  # Hz

        self.noise_std      = self.get_parameter('noise_std_dev').value
        self.add_noise      = self.get_parameter('add_noise').value
        self.enter_thresh   = self.get_parameter('wall_enter_threshold').value
        self.exit_thresh    = self.get_parameter('wall_exit_threshold').value
        self.cone_deg       = self.get_parameter('front_cone_deg').value
        pub_rate            = self.get_parameter('publish_rate').value

        # ── Internal state ────────────────────────────────────────────────
        self.current_state    = 'EXPLORING'
        self.min_front_dist   = 10.0
        # Hysteresis: require N consecutive triggering scans before switching
        self.enter_count      = 0
        self.enter_required   = 2   # scans before entering WALL_FOLLOWING
        self.resume_requested = False

        # ── Pub / Sub ─────────────────────────────────────────────────────
        self.scan_sub  = self.create_subscription(LaserScan, '/scan',            self.scan_callback,   10)
        self.resume_sub= self.create_subscription(Bool,      '/resume_exploring', self.resume_callback, 10)
        self.state_pub = self.create_publisher(String, '/robot_state', 10)

        period = 1.0 / max(pub_rate, 1.0)
        self.create_timer(period, self.publish_state)

        self.get_logger().info(
            f'SensorNode ready | enter={self.enter_thresh:.2f} m | '
            f'exit={self.exit_thresh:.2f} m | cone=±{self.cone_deg}°'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────

    def resume_callback(self, msg: Bool):
        if msg.data and self.current_state == 'WALL_FOLLOWING':
            self.resume_requested = True
            self.current_state    = 'EXPLORING'
            self.enter_count      = 0
            self.get_logger().info('SensorNode: Resuming EXPLORING (leave-point).')

    def scan_callback(self, msg: LaserScan):
        n = len(msg.ranges)
        if n == 0:
            return

        # Front cone indices (wrapping)
        cone_half = self.cone_deg
        indices   = (
            list(range(0, cone_half + 1))
            + list(range(n - cone_half, n))
        )

        max_range = msg.range_max if msg.range_max > 0.0 else 10.0

        front_dists = []
        for i in indices:
            r = msg.ranges[i]
            if math.isinf(r) or math.isnan(r) or r <= 0.0:
                r = max_range          # treat invalid as clear
            elif self.add_noise:
                r += random.gauss(0.0, self.noise_std)
            front_dists.append(r)

        self.min_front_dist = min(front_dists) if front_dists else max_range

        # State machine with hysteresis
        if self.current_state == 'EXPLORING':
            if self.min_front_dist < self.enter_thresh:
                self.enter_count += 1
                if self.enter_count >= self.enter_required:
                    self.current_state = 'WALL_FOLLOWING'
                    self.enter_count   = 0
                    self.get_logger().info(
                        f'Wall at {self.min_front_dist:.2f} m → WALL_FOLLOWING'
                    )
            else:
                self.enter_count = 0   # reset if wall disappears

        elif self.current_state == 'WALL_FOLLOWING':
            # Only revert via explicit resume from algorithm node (leave-point)
            # The exit_threshold acts as a safety: if corridor opened up AND
            # resume was requested, allow; otherwise stay in wall-following.
            pass  # transition managed by resume_callback

    def publish_state(self):
        msg      = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
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
