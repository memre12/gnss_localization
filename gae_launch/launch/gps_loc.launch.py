from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node


import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    orientation_package = get_package_share_directory('imu_filter_madgwick')
    gnss_poser_package = get_package_share_directory('gnss_poser')

    kalman_node = Node(
        package='kalman_filter_localization',
        executable='ekf_localization_node',
        name='kalman_filter_localization',
        output='screen'
    )

    mag_to_quat_node = Node(
        package='mag_to_quat',
        executable='mag_to_quat',
        name='mag_to_quat',
        output='screen'
    )

    imu_filter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orientation_package, 'launch', 'imu_filter_component.launch.py')
        )
    )

    gnss_poser_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(gnss_poser_package, 'launch', 'gnss_poser.launch.xml')
        )
    )

    return LaunchDescription([
        imu_filter_launch,
        gnss_poser_launch,
        kalman_node,
        mag_to_quat_node,
    ])
