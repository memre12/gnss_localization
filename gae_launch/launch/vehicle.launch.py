from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os


def generate_launch_description():
    pkg_name = 'gae_launch'
    launch_dir = os.path.join(get_package_share_directory(pkg_name), 'launch')

    args = [
        DeclareLaunchArgument('use_static_tf', default_value='true'),
        DeclareLaunchArgument('use_imu', default_value='true'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_lidar_container', default_value='true'),
        DeclareLaunchArgument('use_map', default_value='true'),
        DeclareLaunchArgument('use_localization', default_value='true'),
        DeclareLaunchArgument('use_eagleye', default_value='true'),
        DeclareLaunchArgument('use_lidar_perception', default_value='true'),
        DeclareLaunchArgument('map_z', default_value='82.72'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
    ]

    includes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'static_tf.launch.py')),
            condition=IfCondition(LaunchConfiguration('use_static_tf'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'imu.launch.py')),
            condition=IfCondition(LaunchConfiguration('use_imu'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'lidar.launch.py')),
            condition=IfCondition(LaunchConfiguration('use_lidar'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'lidar_container.launch.py')),
            condition=IfCondition(LaunchConfiguration('use_lidar_container'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'map.launch.py')),
            condition=IfCondition(LaunchConfiguration('use_map'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization.launch.py')),
            condition=IfCondition(LaunchConfiguration('use_localization'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'lidar_perception.launch.py')),
            condition=IfCondition(LaunchConfiguration('use_lidar_perception'))
        ),
    ]

    eagleye_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(get_package_share_directory('eagleye_rt'), 'launch', 'eagleye_rt.launch.xml')
        ),
        condition=IfCondition(LaunchConfiguration('use_eagleye')),
    )

    pose_initializer = Node(
        package='gae_pose_initializer_v2',
        executable='gae_pose_initializer_v2',
        name='pose_initializer',
        output='screen',
        parameters=[{
            'map_z': LaunchConfiguration('map_z')
        }]
    )

    robot_description_path = os.path.join(
        get_package_share_directory('gae_launch'), 
        'urdf', 'sensors.urdf'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', robot_description_path])
        }]
    )

    rviz_config_file = os.path.join(
        get_package_share_directory(pkg_name),
        'rviz',
        'localization.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription(args + includes + [
        eagleye_launch,
        pose_initializer,
        robot_state_publisher,
        rviz_node
    ])
