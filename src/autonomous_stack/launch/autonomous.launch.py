from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the realsense launch file
    realsense_launch_dir = get_package_share_directory('realsense2_camera')
    realsense_launch_path = os.path.join(realsense_launch_dir, 'launch', 'rs_pointcloud.launch.py')

    return LaunchDescription([
        # Include RealSense pointcloud launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch_path),
        ),

        # Input nodes
        Node(
            package='autonomous_stack',
            executable='input1a',
            name='input1a',
            output='screen'
        ),

        Node(
            package='autonomous_stack',
            executable='input1b',
            name='input1b',
            output='screen'
        ),

        Node(
            package='autonomous_stack',
            executable='input2',
            name='input2',
            output='screen'
        ),

        # Controller node
        Node(
            package='autonomous_stack',
            executable='controller',
            name='controller',
            output='screen'
        ),

        # Detection nodes
        Node(
            package='autonomous_stack',
            executable='obstacle_detection',
            name='obstacle_detection',
            output='screen'
        ),

        Node(
            package='autonomous_stack',
            executable='pit_detection',
            name='pit_detection',
            output='screen'
        ),

        # Avoidance node
        Node(
            package='autonomous_stack',
            executable='avoidance',
            name='avoidance',
            output='screen'
        ),

        # Map node
        Node(
            package='autonomous_stack',
            executable='map',
            name='map',
            output='screen'
        ),
    ])