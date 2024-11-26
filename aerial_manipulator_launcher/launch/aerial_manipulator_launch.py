from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    propulsion_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('propulsion_controller'), '/launch/propulsion_control_launch.py'
        ])
    )

    winch_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('winch_controller'), '/launch/winch_control_launch.py'
        ])
    )
    oscillation_damping_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('oscillation_damping_controller'), '/launch/oscillation_damping_control_launch.py'
        ])
    )
    yaw_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('yaw_controller'), '/launch/yaw_control_launch.py'
        ])
    )

    return LaunchDescription([
        propulsion_controller_launch,
        winch_controller_launch,
        oscillation_damping_controller_launch,
        yaw_controller_launch,
    ])
