from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource,AnyLaunchDescriptionSource
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ekf_localizer_package = get_package_share_directory('ekf_localizer')
    gyro_odometer_package = get_package_share_directory('gyro_odometer')
    gnss_poser_package = get_package_share_directory('gnss_poser')
    map_launch_package = get_package_share_directory('gae_launch')
    rviz_package = get_package_share_directory('rviz2')
    rviz_config_file = os.path.join(map_launch_package, "rviz", "localisation.rviz")
    
    ekf_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(ekf_localizer_package, 'launch', 'ekf_localizer.launch.xml')
        )
    )

    gyro_odometer_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(  # Assuming it's a .launch.xml but using Python parser works
            os.path.join(gyro_odometer_package, 'launch', 'gyro_odometer.launch.xml')
        )
    )

    gnss_poser_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gnss_poser_package, 'launch', 'gnss_poser.launch.py')
        )
    )

    map_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(map_launch_package, 'launch', 'map.launch.py')
        )
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(rviz_package, rviz_config_file)],
        parameters=[{'use_sim_time': False}],
    )
    

    return LaunchDescription([
        ekf_launch,
        gyro_odometer_launch,
        gnss_poser_launch,
        map_launch,
        rviz_node
    ])
