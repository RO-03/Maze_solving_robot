#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import matplotlib.pyplot as plt
import csv

class LoggerNode(Node):
    def __init__(self):
        super().__init__('logger_node')
        
        self.create_subscription(String, '/performance_log', self.log_callback, 10)
        self.declare_parameter('maze_name', 'simple')
        self.maze_name = self.get_parameter('maze_name').value
        
        # Resolve results directory relative to the current ROS working dir
        self.results_dir = os.path.join(os.getcwd(), 'results')
        if not os.path.exists(self.results_dir):
            os.makedirs(self.results_dir)
            
        self.csv_file = os.path.join(self.results_dir, 'experiment_results.csv')
        
        # Create CSV header if it doesn't exist
        if not os.path.exists(self.csv_file):
            with open(self.csv_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Maze', 'Algorithm', 'Time', 'PathLength', 'Collisions', 'ModeSwitches'])
                
        self.get_logger().info("Logger Node active. Waiting for performance logs...")

    def log_callback(self, msg):
        # Format expects: ALGORITHM,time_taken,path_length,wall_collisions,mode_switches
        data = msg.data.split(',')
        algorithm = data[0]
        time_taken = float(data[1])
        path_length = float(data[2])
        collisions = int(data[3])
        switches = int(data[4])
        
        self.get_logger().info(f"Log received: {algorithm} on {self.maze_name} -> Time: {time_taken}s, Path: {path_length}m")
        
        # Append to CSV
        with open(self.csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([self.maze_name, algorithm, time_taken, path_length, collisions, switches])
            
        # Update Plot
        self.generate_plots()

    def generate_plots(self):
        # Read all data
        data = []
        try:
            with open(self.csv_file, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    data.append(row)
        except Exception as e:
            self.get_logger().error(f"Failed to read CSV for plotting: {e}")
            return

        if not data: return

        # Group data by maze and algorithm
        # Dict structure: maze_data[maze_name][Algo] = metrics
        maze_data = {}
        for d in data:
            maze = d['Maze']
            algo = d['Algorithm']
            if maze not in maze_data:
                maze_data[maze] = {}
            maze_data[maze][algo] = {
                'Time': float(d['Time']),
                'PathLength': float(d['PathLength'])
            }

        mazes = list(maze_data.keys())
        x = range(len(mazes))
        
        hybrid_times = [maze_data[m].get('HYBRID', {}).get('Time', 0) for m in mazes]
        bug2_times = [maze_data[m].get('BASELINE', {}).get('Time', 0) for m in mazes]
        
        hybrid_paths = [maze_data[m].get('HYBRID', {}).get('PathLength', 0) for m in mazes]
        bug2_paths = [maze_data[m].get('BASELINE', {}).get('PathLength', 0) for m in mazes]

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        
        width = 0.35
        # Plot Times
        ax1.bar([i - width/2 for i in x], hybrid_times, width, label='Hybrid Bug2', color='green')
        ax1.bar([i + width/2 for i in x], bug2_times, width, label='Plain Bug2', color='orange')
        ax1.set_ylabel('Time to Solve (s)')
        ax1.set_title('Performance Comparison: Time')
        ax1.set_xticks(x)
        ax1.set_xticklabels(mazes)
        ax1.legend()

        # Plot Path Lengths
        ax2.bar([i - width/2 for i in x], hybrid_paths, width, label='Hybrid Bug2', color='green')
        ax2.bar([i + width/2 for i in x], bug2_paths, width, label='Plain Bug2', color='orange')
        ax2.set_ylabel('Total Path Length (m)')
        ax2.set_title('Performance Comparison: Path Length')
        ax2.set_xticks(x)
        ax2.set_xticklabels(mazes)
        ax2.legend()
        
        plt.tight_layout()
        plot_path = os.path.join(self.results_dir, 'performance_charts.png')
        plt.savefig(plot_path)
        plt.close()
        self.get_logger().info(f"Plots saved to {plot_path}")

def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
