from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='oscillation_damping_controller',
            namespace='Aerial_Manipulator',
            executable='oscillation_damping_control_node',
            name='oscillation_damping_control_node'
        )
    ])