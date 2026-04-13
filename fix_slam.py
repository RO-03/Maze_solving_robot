import os
import glob
import re

files = glob.glob('maze_robot/launch/*.py')

for fpath in files:
    with open(fpath, 'r') as f:
        content = f.read()

    # 1. Add imports if missing
    if "IncludeLaunchDescription" not in content:
        content = content.replace("from launch.actions import ExecuteProcess, TimerAction",
                                  "from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription\nfrom launch.launch_description_sources import PythonLaunchDescriptionSource")

    # 2. Replace the slam node definition
    slam_pattern = r'    slam = Node\([\s\S]*?\n    \)\n'
    
    new_slam = """    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                         'launch', 'online_async_launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )\n"""

    new_content = re.sub(slam_pattern, new_slam, content)

    if new_content != content:
        with open(fpath, 'w') as f:
            f.write(new_content)
        print(f"Fixed {fpath}")
    else:
        print(f"No changes made to {fpath}")

