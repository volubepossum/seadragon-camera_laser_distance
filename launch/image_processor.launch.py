from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('distance_measure')
    
    config_file = os.path.join(pkg_dir, 'config', 'image_processor_params.yaml')
    
    return LaunchDescription([
        Node(
            package='image_processor',
            executable='image_processor_node',
            name='distance_measurement',
            parameters=[config_file],
            output='screen',
        ),
    ])