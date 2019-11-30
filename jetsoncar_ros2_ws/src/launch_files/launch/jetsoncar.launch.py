import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='racecar_control',
            node_executable='control_mux',
            node_name='control_mux',
            parameters=['src/racecar_control/config/mux_config.yaml']
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
            parameters=['src/ai_drivers/config/data_collector_config.yaml']
        )
    ])