from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([

        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value='/root/ROS2Activities/workspace/challenge3_ws/install/puzzlebot_gazebo/share/puzzlebot_gazebo/model:${GAZEBO_MODEL_PATH}'
        ),
        # Puzzlebot parameters
        DeclareLaunchArgument('robot_name', default_value='puzzlebot_1'),
        DeclareLaunchArgument('robot_description_file', default_value='puzzlebot_jetson_lidar_ed_v1.xacro'),
        DeclareLaunchArgument('pos_x', default_value='2.0'),
        DeclareLaunchArgument('pos_y', default_value='2.0'),
        DeclareLaunchArgument('pos_theta', default_value='-1.57'),
        
        # Gazebo parameters
        DeclareLaunchArgument('world_name', default_value=PathJoinSubstitution(
            [FindPackageShare('puzzlebot_gazebo'), 'worlds', 'puzzlebot_arena_markers.world'])),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('debug', default_value='false'),

        # Include Gazebo launch file
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
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('gazebo_ros'), 'launch', 'gzclient.launch.py'])),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('puzzlebot_gazebo'), 'launch', 'puzzlebot_spawner.launch.py'])),
            launch_arguments={
                'robot': LaunchConfiguration('robot_name'),
                'robot_description_file': LaunchConfiguration('robot_description_file'),
                'x': LaunchConfiguration('pos_x'),
                'y': LaunchConfiguration('pos_y'),
                'yaw': LaunchConfiguration('pos_theta')
            }.items(),
        ),
    ])
