from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='propulsion_controller',
            namespace='aerial_platform',
            executable='propulsion_control_node',
            name='propulsion_control_node'
        ),
    ])