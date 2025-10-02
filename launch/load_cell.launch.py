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
    
    # Declare launch arguments
    # publish_rate_arg = DeclareLaunchArgument(
    #     'publish_rate',
    #     default_value='20.0',
    #     description='Publishing rate in Hz'
    # )
    # 
    # average_samples_arg = DeclareLaunchArgument(
    #     'average_samples',
    #     default_value='5',
    #     description='Number of samples to average for each reading'
    # )
    # 
    # calibration_file_arg = DeclareLaunchArgument(
    #     'calibration_file',
    #     default_value='load_cell_calibration.json',
    #     description='Calibration file name'
    # )
    
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
        # publish_rate_arg,
        # average_samples_arg,
        # calibration_file_arg,
        config_file_arg,
        load_cell_node,
    ])
