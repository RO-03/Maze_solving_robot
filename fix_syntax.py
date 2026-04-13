import os
import glob
import re

files = glob.glob('maze_robot/maze_robot/*_node.py')

broken_pattern = r'    try:\n        try:\n        rclpy\.spin\((.+?)\)\n    except KeyboardInterrupt:\n        pass\n    finally:\n    except KeyboardInterrupt:\n        pass\n    finally:\n(.*?)if rclpy\.ok\(\):\n        rclpy\.shutdown\(\)'

for fpath in files:
    with open(fpath, 'r') as f:
        content = f.read()

    def replacer(match):
        node_var = match.group(1)
        middle = match.group(2)
        return f"    try:\n        rclpy.spin({node_var})\n    except KeyboardInterrupt:\n        pass\n    finally:{middle}        if rclpy.ok():\n            rclpy.shutdown()"

    new_content, count = re.subn(broken_pattern, replacer, content, flags=re.DOTALL)
    
    if count > 0:
        with open(fpath, 'w') as f:
            f.write(new_content)
        print(f"Fixed syntax in {fpath}")
    else:
        print(f"No syntax match in {fpath}")

