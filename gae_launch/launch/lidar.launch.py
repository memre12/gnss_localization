import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import yaml

def generate_launch_description():
    velodyne_driver_param_path = os.path.join(
        get_package_share_directory('gae_launch'),
        'config',
        'lidar',
        'velodyne_driver.param.yaml'
    )

    velodyne_driver_node = Node(
        package = 'velodyne_driver',
        executable = 'velodyne_driver_node',
        output = 'both',
        parameters = [velodyne_driver_param_path]
    )


    velodyne_transform_param_path = os.path.join(
        get_package_share_directory('gae_launch'),
        'config',
        'lidar',
        'velodyne_transform.param.yaml'
    )

    with open(velodyne_transform_param_path, 'r') as f:
        velodyne_transform_config = yaml.safe_load(f)
    
    velodyne_transform_config['calibration'] = os.path.join(
        get_package_share_directory('gae_launch'),
        'config',
        'lidar',
        'velodyne_calibration.yaml'
    )

    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/sensing/lidar_top/points_raw',
        description='Lidar raw points topic name'
    )

    velodyne_transform_node = Node(
        package = 'velodyne_pointcloud',
        executable = 'velodyne_transform_node',
        output = 'both',
        parameters = [velodyne_transform_config],
        remappings = [
            ('/velodyne_points', LaunchConfiguration('lidar_topic')),
        ]
    )

    passthrough_filter_param_path = os.path.join(
        get_package_share_directory('gae_launch'),
        'config',
        'lidar',
        'passthrough_filter_node.param.yaml'
    )

    output_passthrough_filtered_points_topic_arg = DeclareLaunchArgument(
        'output_passthrough_filtered_points_topic',
        default_value='/sensing/lidar_top/points_filtered',
        description='Output passthrough filtered points topic name'
    )

    passthrough_filter_node = Node(
        package='pointcloud_preprocessor',
        executable='passthrough_filter_uint16_node',
        output='both',
        parameters=[passthrough_filter_param_path],
        remappings=[
            ('input', LaunchConfiguration('lidar_topic')),
            ('output', LaunchConfiguration('output_passthrough_filtered_points_topic'))
        ])


    distortion_corrector_param_path = os.path.join(
        get_package_share_directory('gae_launch'),
        'config',
        'lidar',
        'distortion_corrector_node.param.yaml'
    )

    input_twist_topic_arg = DeclareLaunchArgument(
        'input_twist_topic',
        default_value='/sbg/ros/twist_with_covariance_stamped',
        description='Input twist topic name'
    )
    input_imu_topic_arg = DeclareLaunchArgument(
        'input_imu_topic',
        default_value='/sbg/ros/imu/data',
        description='Input IMU topic name'
    )

    output_corrected_points_topic_arg = DeclareLaunchArgument(
        'output_corrected_points_topic',
        default_value='/sensing/lidar_top/points_corrected',
        description='Output corrected points topic name'
    )

    distortion_corrector_node = Node(
        package = 'pointcloud_preprocessor',
        executable = 'distortion_corrector_node',
        output = 'both',
        parameters = [distortion_corrector_param_path],
        remappings = [
            ('~/input/pointcloud', LaunchConfiguration('output_passthrough_filtered_points_topic')),
            ('~/input/twist', LaunchConfiguration('input_twist_topic')),
            ('~/input/imu', LaunchConfiguration('input_imu_topic')),
            ('~/output/pointcloud', LaunchConfiguration('output_corrected_points_topic'))
        ]
    )


    ring_outlier_filter_param_path = os.path.join(
        get_package_share_directory('gae_launch'),
        'config',
        'lidar',
        'ring_outlier_filter.param.yaml'
    )

    output_ring_outlier_filter_topic_arg = DeclareLaunchArgument(
        'output_ring_outlier_filter_topic',
        default_value='/sensing/lidar_top/points',
        description='Output ring outlier filter topic name'
    )

    output_ring_outlier_filter_debug_topic_arg = DeclareLaunchArgument(
        'output_ring_outlier_filter_debug_topic',
        default_value='/sensing/lidar_top/ring_outlier_filter/debug',
        description='Output ring outlier filter debug topic name'
    )

    ring_outlier_filter_debug_visiblity_topic_arg = DeclareLaunchArgument(
        'ring_outlier_filter_debug_visiblity_topic',
        default_value='/sensing/lidar_top/ring_outlier_filter/debug_visibility',
        description='Ring outlier filter debug visibility topic'
    )

    ring_outlier_filter_node = Node(
        package = 'pointcloud_preprocessor',
        executable = 'ring_outlier_filter_node',
        output = 'both',
        parameters = [ring_outlier_filter_param_path],
        remappings = [
            ('/input', LaunchConfiguration('output_corrected_points_topic')),
            ('/output', LaunchConfiguration('output_ring_outlier_filter_topic')),
            ('/debug/ring_outlier_filter', LaunchConfiguration('output_ring_outlier_filter_debug_topic')),
            ('/ring_outlier_filter/debug/visibility', LaunchConfiguration('ring_outlier_filter_debug_visiblity_topic'))
        ]
    )

    return LaunchDescription([
        velodyne_driver_node,
        lidar_topic_arg,
        velodyne_transform_node,
        output_passthrough_filtered_points_topic_arg,
        passthrough_filter_node,
        input_twist_topic_arg,
        input_imu_topic_arg,
        output_corrected_points_topic_arg,
        distortion_corrector_node,
        output_ring_outlier_filter_topic_arg,
        output_ring_outlier_filter_debug_topic_arg,
        ring_outlier_filter_debug_visiblity_topic_arg,
        ring_outlier_filter_node
    ])