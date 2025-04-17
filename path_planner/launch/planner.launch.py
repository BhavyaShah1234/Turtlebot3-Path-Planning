import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    turtlebot3_nav2 = os.path.join(
        get_package_share_directory('turtlebot3_navigation2'),
        'launch',
        'navigation2.launch.py'
    )

    map_file = os.path.join(
        get_package_share_directory('path_planner'),
        'maps',
        'map_house.yaml'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(turtlebot3_nav2),
            launch_arguments={'map': map_file}.items()
        ),
        Node(
            package='turtlebot3_goal_nav',
            executable='goal_publisher',
            name='executor',
            output='screen'
        )
    ])
