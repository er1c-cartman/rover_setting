from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='four_wheel_robot_control',
            executable='four_wheel_kinematics',
            name='four_wheel_kinematics',
            output='screen',
            parameters=['config/four_wheel_kinematics.yaml']
        )
    ])

