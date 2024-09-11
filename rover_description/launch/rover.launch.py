import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Generate URDF from Xacro
    xacro_file_path = PathJoinSubstitution([
        FindPackageShare("rover_description"), "urdf", "rover.xacro"
    ])
    
    robot_description_content = Command([
        'xacro ', xacro_file_path
    ])

    # Node for robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # Node for the fake odometry publisher
    fake_odom_publisher_node = Node(
        package='rover_description',
        executable='fake_odom_publisher',
        output='screen'
    )

    # Node for RViz
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare("rover_description"), "rviz", "rover_config.rviz"
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    # Node for four_wheel_kinematics
    four_wheel_kinematics_node = Node(
        package='four_wheel_robot_control',
        executable='four_wheel_kinematics',
        name='four_wheel_kinematics',
        output='screen',
        parameters=[PathJoinSubstitution([
            FindPackageShare("four_wheel_robot_control"), "config", "four_wheel_kinematics.yaml"
        ])]
    )

    # Node for velocity control using dynamixel_sdk_examples
    velocity_control_node = Node(
        package='dynamixel_sdk_examples',
        executable='velocity_control_node',
        name='velocity_control_node',
        output='screen'
    )

    # Node for AHRS
    ahrs_driver_node = Node(
        package="fdilink_ahrs",
        executable="ahrs_driver_node",
        parameters=[{
            'if_debug_': False,
            'serial_port_': '/dev/ttyUSB1',
            'serial_baud_': 921600,
            'imu_topic': '/imu',
            'imu_frame_id_': 'gyro_link',
            'mag_pose_2d_topic': '/mag_pose_2d',
            'Magnetic_topic': '/magnetic',
            'Euler_angles_topic': '/euler_angles',
            'gps_topic': '/gps/fix',
            'twist_topic': '/system_speed',
            'NED_odom_topic': '/NED_odometry',
            'device_type_': 1
        }],
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher_node,
        fake_odom_publisher_node,
        rviz_node,
        four_wheel_kinematics_node,
        velocity_control_node,
        ahrs_driver_node,  # Added AHRS node
    ])

