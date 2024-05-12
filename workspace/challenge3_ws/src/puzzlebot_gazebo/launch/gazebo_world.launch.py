from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value=PathJoinSubstitution(
            [FindPackageShare('puzzlebot_gazebo'), 'worlds', 'obstacle_avoidance_1.world'])),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('debug', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('gazebo_ros'), 'launch', 'gzserver.launch.py'])),
            launch_arguments={
                'world': LaunchConfiguration('world_name'),
                'paused': LaunchConfiguration('paused'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'gui': LaunchConfiguration('gui'),
                'headless': LaunchConfiguration('headless'),
                'debug': LaunchConfiguration('debug'),
                'extra_gazebo_args': '--lockstep'
            }.items(),
        )
    ])
