
from launch import LaunchDescription
import launch_ros.actions



def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_vision', executable='robot_vision_node', output='screen'),#, prefix='xterm -e'),
        launch_ros.actions.Node(
            package='v4l2_camera', executable='v4l2_camera_node', output='screen', prefix='xterm -e'),
        launch_ros.actions.Node(
            package='mg996r', executable='mg996r_node', output='screen', prefix='xterm -e'),
        launch_ros.actions.Node(
            package='test_kinematics', executable='test_kinematics_node', output='screen', prefix='xterm -e'),
        launch_ros.actions.Node(
            package='robot_data', executable='robot_data_node', output='screen', prefix='xterm -e'),
        ])
