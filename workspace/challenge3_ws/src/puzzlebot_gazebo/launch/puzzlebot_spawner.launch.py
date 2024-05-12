from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, GroupAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='puzzlebot_1'),
        DeclareLaunchArgument('robot_description_file', default_value='puzzlebot_hacker_ed_v1_3.xacro'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('wr_topic', default_value='wr'),
        DeclareLaunchArgument('wl_topic', default_value='wl'),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='puzzlebot_state_publisher',
            namespace=LaunchConfiguration('robot'),
            parameters=[{
                'robot_description': Command([
                    FindExecutable(name='xacro'),
                    ' ',
                    PathJoinSubstitution([
                        FindPackageShare('puzzlebot_description'),
                        'urdf',
                        LaunchConfiguration('robot_description_file')
                    ]),
                    ' prefix:=',
                    LaunchConfiguration('robot')
                ]),
            }],
            remappings=[
                ('robot_description', [LaunchConfiguration('robot'), '/robot_description'])
            ],
        ),

        # Puzzlebot spawner
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', LaunchConfiguration('robot'),
                '-topic', 'robot_description',
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
                '-Y', LaunchConfiguration('yaw')
            ],
            output='screen'
        ),

        # Include the robot controller launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('puzzlebot_control'), 'launch', 'puzzlebot_control.launch.py'])),
            launch_arguments={'robot': LaunchConfiguration('robot')}.items(),
        )
    ])
