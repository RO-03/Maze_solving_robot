import os
import glob
import re

files = glob.glob('maze_robot/maze_robot/*_node.py')

for fpath in files:
    with open(fpath, 'r') as f:
        content = f.read()

    # Extract filename without .py
    fname = os.path.basename(fpath).replace('.py', '')
    
    # Generate PascalCase class name
    class_name = ''.join(word.capitalize() for word in fname.split('_'))

    new_main = f"""def main(args=None):
    rclpy.init(args=args)
    node = {class_name}()
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
"""

    # Replace everything from def main(args=None): onwards
    pattern = r'def main\(args=None\):.*'
    new_content = re.sub(pattern, new_main, content, flags=re.DOTALL)

    with open(fpath, 'w') as f:
        f.write(new_content)
    print(f"Fixed {fpath}")

