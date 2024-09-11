import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation/Gazebo clock')


    nav2_config_dir = LaunchConfiguration(
        'config_file',
        default=os.path.join(
            get_package_share_directory('rover_navigation'),
            'config',
            'mppi_nav2.yaml'))
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    nav2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch_file_dir, '/navigation_launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_config_dir}.items(),
    )



    occmap_to_costmap_node = Node(
        package='rover_navigation',
        executable='occmap_to_costmap',
        name='occmap_to_costmap',
        output='screen')

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(nav2_node)

    ld.add_action(occmap_to_costmap_node)

    return ld

