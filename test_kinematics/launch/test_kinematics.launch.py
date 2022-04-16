
from launch import LaunchDescription
import launch_ros.actions



def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_data', node_executable='robot_data_node', output='screen', prefix='xterm -e'),
        launch_ros.actions.Node(
            package='robot_vision', node_executable='robot_vision_node', output='screen', prefix='xterm -e'),
        launch_ros.actions.Node(
            package='v4l2_camera', node_executable='v4l2_camera_node', output='screen', prefix='xterm -e'),
        launch_ros.actions.Node(
            package='mg996r', node_executable='mg996r_node', output='screen', prefix='xterm -e'),
        launch_ros.actions.Node(
            package='test_kinematics', node_executable='test_kinematics_node', output='screen'),
        ])
