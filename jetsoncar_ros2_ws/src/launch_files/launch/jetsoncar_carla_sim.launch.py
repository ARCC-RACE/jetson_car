import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix
import os


def generate_launch_description():
    #  https://github.com/carla-simulator/ros-bridge
    print("Make sure to run a roscore in a terminal with only ROS melodic sourced. Carla (version 0.9.7.4) should be run with `./CarlaUE4.sh -carla-server -benchmark -fps=15 -windowed -ResX=800 -ResY=600` and start the carla ROS bridge in a melodic terminal with `export PYTHONPATH=$PYTHONPATH:/media/michael/BigMemory/carla/CARLA_0.9.7.4/PythonAPI/carla/dist/carla-0.9.7-py2.7-linux-x86_64.egg` and `roslaunch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch`")

    return launch.LaunchDescription([
        Node(
            package='racecar_control',
            node_executable='control_mux',
            node_name='control_mux',
            parameters=[os.path.join(get_package_prefix('racecar_control'),
                                     '../../src/racecar_control/config/mux_config.yaml')]
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
            package='ai_drivers',
            node_executable='model_loader',
            node_name='model_loader',
            output='screen',
            parameters=[os.path.join(get_package_prefix('ai_drivers'),
                                     '../../src/ai_drivers/config/model_loader_config.yaml')]
        ),
        Node(
            package='ros1_bridge',
            node_executable='dynamic_bridge',
            node_namespace='',
            output='screen'
        ),
        Node(
            package='joy',
            node_executable='joy_node',
            node_namespace='',
            output='screen'
        ),
    ])



