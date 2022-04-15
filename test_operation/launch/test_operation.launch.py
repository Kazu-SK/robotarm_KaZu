
from launch import LaunchDescription
import launch_ros.actions



def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='test_operation', node_executable='test_operation_node', output='screen'),
        launch_ros.actions.Node(
            package='mg996r', node_executable='mg996r_node', output='screen', prefix='xterm -e'),
        ])
