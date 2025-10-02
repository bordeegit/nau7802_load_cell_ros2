from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('nau7802_load_cell_ros2')

    # Path to config file
    config_file = os.path.join(pkg_share, 'config', 'load_cell_params.yaml')

    # config_file = LaunchConfiguration('config_file')
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Path to config file'
    )

    
    # Load cell node
    load_cell_node = Node(
        package='nau7802_load_cell_ros2',
        executable='load_cell_node',
        name='load_cell_node',
        output='screen',
        parameters=[config_file],
        remappings=[
            # Add any topic remappings here if needed
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        load_cell_node,
    ])
