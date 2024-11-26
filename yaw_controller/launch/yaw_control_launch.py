from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yaw_controller',
            namespace='Aerial_Manipulator',
            executable='yaw_control_node',
            name='yaw_control_node'
        )
    ])