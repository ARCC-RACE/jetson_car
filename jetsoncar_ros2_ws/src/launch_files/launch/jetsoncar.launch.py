import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix
import os


def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='racecar_control',
            node_executable='control_mux',
            node_name='control_mux',
            parameters=[os.path.join(get_package_prefix('racecar_control'),
                                     '../../src/racecar_control/config/mux_config.yaml')]
        ),
        Node(
            package='usb_hw_interface',
            node_executable='usb_hw_interface_node',
            node_name='hw_interface'
        ),
        Node(
            package='ai_drivers',
            node_executable='data_collector',
            node_name='data_collector',
            output='screen',
            parameters=[os.path.join(get_package_prefix('ai_drivers'),
                                     '../../src/ai_drivers/config/data_collector_config.yaml')]
        ),
        Node(
            package='realsense_node',
            node_executable='realsense_node',
            node_namespace='',
            output='screen',
            parameters=[os.path.join(get_package_prefix('launch_files'),
                                     '../../src/launch_files/config/realsense_config.yaml')]
        ),
    ])



