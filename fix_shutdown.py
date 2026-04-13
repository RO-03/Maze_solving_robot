import os
import glob
import re

files = glob.glob('maze_robot/maze_robot/*_node.py')

for fpath in files:
    with open(fpath, 'r') as f:
        content = f.read()

    # The typical structure is:
    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()
    
    # Let's replace:
    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()
    # OR variations.
    
    # We can match `rclpy.spin\(.*?\)[\s\S]*?rclpy.shutdown\(\)`
    # But wait, it's safer to just look for the main block.
    # Usually it's:
    # def main(args=None):
    #     rclpy.init(args=args)
    #     node = SensorNode() # or something
    #     rclpy.spin(node)
    #     node.destroy_node()
    #     rclpy.shutdown()
    
    # A robust regex replacement for the spin/shutdown block:
    pattern = r'(rclpy\.spin\(([^)]+)\))(.*?)(rclpy\.shutdown\(\))'
    
    def replacer(match):
        node_var = match.group(2)
        middle = match.group(3)
        return f"try:\n        rclpy.spin({node_var})\n    except KeyboardInterrupt:\n        pass\n    finally:{middle}if rclpy.ok():\n        rclpy.shutdown()"
    
    new_content, count = re.subn(pattern, replacer, content, flags=re.DOTALL)
    
    if count > 0:
        with open(fpath, 'w') as f:
            f.write(new_content)
        print(f"Fixed {fpath}")
    else:
        print(f"Could not automatically fix {fpath}")
