import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'node_prefix',
            default_value=[launch.substitutions.EnvironmentVariable('USER'), '_'],
            description='Prefix for node names'),
        launch_ros.actions.Node(
            package='racecar_control', node_executable='control_mux', output='screen',
            node_name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'control_mux'],
            parameters='config/mux_config.yaml'),
    ])