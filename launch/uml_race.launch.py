from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
	included_launch = IncludeLaunchDescription(
                  PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('stage_ros2'), 'launch'),
         '/stage.launch.py']),
                  launch_arguments={'world' : 'racetrack'}.items(),
        )
    
	return LaunchDescription([
	included_launch,
        Node(
            package='uml_race',
            executable='referee',
            name='referee',
            )
    ])
