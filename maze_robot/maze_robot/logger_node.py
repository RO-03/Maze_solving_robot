#!/usr/bin/env python3
"""
logger_node.py  –  Production-grade Performance Logger
=======================================================
Improvements over the MVP:
  1. Safe CSV parsing with try/except on every row.
  2. Results directory resolved from XDG_RUNTIME_DIR or HOME, not cwd
     (avoids permission issues when launched by ros2 launch).
  3. Matplotlib uses the 'Agg' backend explicitly to prevent display errors
     when running headless / inside a launch file.
  4. Plot includes a third subplot for collisions.
  5. Exploration-complete event also logged to CSV.
  6. Node shuts down gracefully on KeyboardInterrupt.
"""

import csv
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

# Force non-interactive Matplotlib backend before importing pyplot
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def _results_dir() -> str:
    """Return a writable directory for results, creating it if needed."""
    base = os.environ.get(
        'ROS_LOG_DIR',
        os.path.join(os.environ.get('HOME', '/tmp'), 'maze_robot_results'),
    )
    path = os.path.join(base, 'results')
    os.makedirs(path, exist_ok=True)
    return path


class LoggerNode(Node):
    def __init__(self):
        super().__init__('logger_node')

        self.declare_parameter('maze_name', 'simple')
        self.maze_name = self.get_parameter('maze_name').value

        self.results_dir = _results_dir()
        self.csv_file    = os.path.join(self.results_dir, 'experiment_results.csv')

        # Write header if new file
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, 'w', newline='') as f:
                csv.writer(f).writerow(
                    ['Maze', 'Algorithm', 'Time_s', 'PathLength_m',
                     'Collisions', 'ModeSwitches']
                )

        self.create_subscription(String, '/performance_log',      self.log_callback,  10)
        self.create_subscription(Bool,   '/exploration_complete', self.done_callback, 10)

        self.get_logger().info(
            f'LoggerNode ready | maze={self.maze_name} | '
            f'results → {self.results_dir}'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────

    def log_callback(self, msg: String):
        try:
            parts      = msg.data.split(',')
            algorithm  = parts[0]
            time_taken = float(parts[1])
            path_len   = float(parts[2])
            collisions = int(parts[3])
            switches   = int(parts[4])
        except (IndexError, ValueError) as e:
            self.get_logger().error(f'Malformed log message "{msg.data}": {e}')
            return

        self.get_logger().info(
            f'{algorithm} on {self.maze_name} → '
            f't={time_taken:.1f}s  d={path_len:.1f}m  '
            f'collisions={collisions}  switches={switches}'
        )

        with open(self.csv_file, 'a', newline='') as f:
            csv.writer(f).writerow(
                [self.maze_name, algorithm, time_taken, path_len,
                 collisions, switches]
            )

        self._generate_plots()

    def done_callback(self, msg: Bool):
        if msg.data:
            self.get_logger().info('Exploration complete event received.')
            with open(self.csv_file, 'a', newline='') as f:
                csv.writer(f).writerow(
                    [self.maze_name, 'EVENT_EXPLORATION_COMPLETE', '', '', '', '']
                )

    # ── Plotting ──────────────────────────────────────────────────────────

    def _generate_plots(self):
        # Read valid data rows
        data = []
        try:
            with open(self.csv_file, newline='') as f:
                for row in csv.DictReader(f):
                    try:
                        data.append({
                            'Maze':        row['Maze'],
                            'Algorithm':   row['Algorithm'],
                            'Time':        float(row['Time_s']),
                            'PathLength':  float(row['PathLength_m']),
                            'Collisions':  int(row['Collisions']),
                        })
                    except (ValueError, KeyError):
                        pass   # skip event / malformed rows
        except Exception as e:
            self.get_logger().error(f'Cannot read CSV for plotting: {e}')
            return

        if not data:
            return

        # Group by maze × algorithm, keep last entry per combo
        grouped: dict[str, dict[str, dict]] = {}
        for d in data:
            maze = d['Maze']
            algo = d['Algorithm']
            if maze not in grouped:
                grouped[maze] = {}
            grouped[maze][algo] = d

        mazes = sorted(grouped.keys())
        x     = list(range(len(mazes)))
        width = 0.35

        def vals(key: str, algo: str):
            return [grouped[m].get(algo, {}).get(key, 0) for m in mazes]

        fig, axes = plt.subplots(1, 3, figsize=(16, 5))
        fig.suptitle('Maze Robot – Algorithm Comparison', fontsize=14)

        metrics = [
            ('Time',       'Time to Solve (s)',    'Performance: Time'),
            ('PathLength', 'Total Path Length (m)', 'Performance: Distance'),
            ('Collisions', 'Wall Collisions',       'Performance: Safety'),
        ]

        for ax, (key, ylabel, title) in zip(axes, metrics):
            ax.bar(
                [i - width / 2 for i in x], vals(key, 'HYBRID'),
                width, label='Hybrid Bug2', color='#2ecc71',
            )
            ax.bar(
                [i + width / 2 for i in x], vals(key, 'BASELINE'),
                width, label='Plain Bug2', color='#e67e22',
            )
            ax.set_ylabel(ylabel)
            ax.set_title(title)
            ax.set_xticks(x)
            ax.set_xticklabels(mazes, rotation=15, ha='right')
            ax.legend()
            ax.grid(axis='y', linestyle='--', alpha=0.4)

        plt.tight_layout()
        plot_path = os.path.join(self.results_dir, 'performance_charts.png')
        try:
            plt.savefig(plot_path, dpi=120)
            self.get_logger().info(f'Plot saved → {plot_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to save plot: {e}')
        finally:
            plt.close(fig)


def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
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
