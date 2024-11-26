from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='winch_controller',
            namespace='Aerial_Manipulator',
            executable='winch_control_node',
            name='winch_control_node'
        )
    ])