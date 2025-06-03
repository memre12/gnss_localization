import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ekf_localizer_param_path = os.path.join(
        get_package_share_directory('gae_launch'),
        'config',
        'localization',
        'ekf_localizer.param.yaml'
    )

    input_ndt_pose_with_cov_topic_arg = DeclareLaunchArgument(
        'input_ndt_pose_with_covariance_topic',
        default_value='/localization/ndt_localizer/pose_with_cov',
        description='Input pose with covariance topic name for EKF localizer and output pose with covariance topic name for NDT localizer'
    )

    input_trigger_node_service_topic_arg = DeclareLaunchArgument(
        'input_trigger_node_service_topic',
        default_value='/localization/ekf_localizer/trigger_node_service',
        description='Input trigger node service topic name for EKF localizer'
    )

    input_ndt_pose_with_covariance_topic_arg = DeclareLaunchArgument(
        'input_ndt_pose_with_covariance_topic',
        default_value='/gnss_pose_cov',
        description='Input NDT pose with covariance topic name for EKF localizer'
    )

    input_twist_with_covariance_topic_arg = DeclareLaunchArgument(
        'input_twist_with_covariance_topic',
        default_value='/sbg/ros/twist_with_covariance_stamped',
        description='Input twist with covariance topic name for EKF localizer'
    )

    output_ekf_odom_topic_arg = DeclareLaunchArgument(
        'output_ekf_odom_topic',
        default_value='/localization/ekf_localizer/odom',
        description='Output odometry topic name for EKF localizer'
    )

    output_ekf_pose_topic_arg = DeclareLaunchArgument(
        'output_ekf_pose_topic',
        default_value='/localization/ekf_localizer/pose',
        description='Output pose topic name for EKF localizer'
    )

    output_ekf_pose_with_cov_topic_arg = DeclareLaunchArgument(
        'output_ekf_pose_with_covariance_topic',
        default_value='/localization/ekf_localizer/pose_with_cov',
        description='Output pose with covariance topic name for EKF localizer'
    )

    output_ekf_biased_pose_topic_arg = DeclareLaunchArgument(
        'output_ekf_biased_pose_topic',
        default_value='/localization/ekf_localizer/biased_pose',
        description='Output biased pose topic name for EKF localizer'
    )

    output_ekf_biased_pose_with_cov_topic_arg = DeclareLaunchArgument(
        'output_ekf_biased_pose_with_covariance_topic',
        default_value='/localization/ekf_localizer/biased_pose_with_cov',
        description='Output biased pose with covariance topic name for EKF localizer'
    )

    output_ekf_twist_topic_arg = DeclareLaunchArgument(
        'output_ekf_twist_topic',
        default_value='/localization/ekf_localizer/twist',
        description='Output twist topic name for EKF localizer'
    )

    output_ekf_twist_with_cov_topic_arg = DeclareLaunchArgument(
        'output_ekf_twist_with_covariance_topic',
        default_value='/localization/ekf_localizer/twist_with_cov',
        description='Output twist with covariance topic name for EKF localizer'
    )

    output_ekf_estimated_yaw_bias_topic_arg = DeclareLaunchArgument(
        'output_ekf_estimated_yaw_bias_topic',
        default_value='/localization/ekf_localizer/estimated_yaw_bias',
        description='Output estimated yaw bias topic name for EKF localizer'
    )

    ekf_localizer_node = Node(
        package = 'ekf_localizer',
        executable = 'ekf_localizer_node',
        output = 'both',
        parameters = [ekf_localizer_param_path],
        remappings = [
            ('initialpose', 'initialpose3d'),
            ('trigger_node_srv', LaunchConfiguration('input_trigger_node_service_topic')),
            ('in_pose_with_covariance', LaunchConfiguration('input_ndt_pose_with_covariance_topic')),
            ('in_twist_with_covariance', LaunchConfiguration('input_twist_with_covariance_topic')),
            ('ekf_odom', LaunchConfiguration('output_ekf_odom_topic')),
            ('ekf_pose', LaunchConfiguration('output_ekf_pose_topic')),
            ('ekf_pose_with_covariance', LaunchConfiguration('output_ekf_pose_with_covariance_topic')),
            ('ekf_biased_pose', LaunchConfiguration('output_ekf_biased_pose_topic')),
            ('ekf_biased_pose_with_covariance', LaunchConfiguration('output_ekf_biased_pose_with_covariance_topic')),
            ('ekf_twist', LaunchConfiguration('output_ekf_twist_topic')),
            ('ekf_twist_with_covariance', LaunchConfiguration('output_ekf_twist_with_covariance_topic')),
            ('estimated_yaw_bias', LaunchConfiguration('output_ekf_estimated_yaw_bias_topic'))
        ]
    )

    ndt_scan_matcher_param_path = os.path.join(
        get_package_share_directory('gae_launch'),
        'config',
        'localization',
        'ndt_scan_matcher.param.yaml'
    )
    input_pointcloud_topic_arg = DeclareLaunchArgument(
        'input_pointcloud_topic',
        default_value='/sensing/lidar_top/points_downsampled',
        description='Input pointcloud topic name for NDT localizer'
    )

    input_gnss_pose_with_cov_topic_arg = DeclareLaunchArgument(
        'input_gnss_pose_with_cov_topic',
        default_value='/sensing/gnss/gnss_pose_with_cov',
        description='Input GNSS pose with covariance topic name for NDT localizer'
    )

    input_ndt_trigger_node_service_topic_arg = DeclareLaunchArgument(
        'input_ndt_trigger_node_service_topic',
        default_value='/localization/ndt_localizer/trigger_node_service',
        description='Input trigger node service topic name for NDT localizer'
    )

    input_ndt_align_service_topic_arg = DeclareLaunchArgument(
        'input_ndt_align_service_topic',
        default_value='/localization/ndt_localizer/align_service',
        description='Input align service topic name for NDT localizer'
    )

    output_ndt_pose_topic_arg = DeclareLaunchArgument(
        'output_ndt_pose_topic',
        default_value='/localization/ndt_localizer/pose',
        description='Output pose topic name for NDT localizer'
    )
    output_ndt_initial_pose_topic_arg = DeclareLaunchArgument(
        'output_ndt_initial_pose_topic',
        default_value='/localization/ndt_localizer/initial_pose',
        description='Output initial pose topic name for NDT localizer'
    )

    output_ndt_marker_topic_arg = DeclareLaunchArgument(
        'output_ndt_marker_topic',
        default_value='/localization/ndt_localizer/marker',
        description='Output marker topic name for NDT localizer'
    )

    input_client_map_loader_service_topic_arg = DeclareLaunchArgument(
        'input_client_map_loader_service_topic',
        default_value='/map/get_differential_pointcloud_map',
        description='Input client map loader service topic name for NDT localizer'
    )

    output_debug_loaded_map_topic_arg = DeclareLaunchArgument(
        'output_debug_loaded_map_topic',
        default_value='/localization/ndt_localizer/debug/loaded_map',
        description='Output debug load map topic name for NDT localizer'
    )

    output_debug_exe_time_topic_arg = DeclareLaunchArgument(
        'output_exe_time_topic',
        default_value='/localization/ndt_localizer/debug/exe_time_ms',
        description='Output execution time topic name for NDT localizer'
    )

    output_debug_initial_to_result_distance_topic_arg = DeclareLaunchArgument(
        'output_initial_to_result_distance_topic',
        default_value='/localization/ndt_localizer/debug/initial_to_result_distance',
        description='Output initial to result distance topic name for NDT localizer'
    )

    output_debug_initial_to_result_distance_new_topic_arg = DeclareLaunchArgument(
        'output_initial_to_result_distance_new_topic',
        default_value='/localization/ndt_localizer/debug/initial_to_result_distance_new',
        description='Output initial to result distance new topic name for NDT localizer'
    )    

    output_debug_initial_to_result_distance_old_topic_arg = DeclareLaunchArgument(
        'output_initial_to_result_distance_old_topic',
        default_value='/localization/ndt_localizer/debug/initial_to_result_distance_old',
        description='Output initial to result distance old topic name for NDT localizer'
    )

    output_debug_initial_to_result_relative_pose_topic_arg = DeclareLaunchArgument(
        'output_initial_to_result_relative_pose_topic',
        default_value='/localization/ndt_localizer/debug/initial_to_result_relative_pose',
        description='Output initial to result relative pose topic name for NDT localizer'
    )

    output_debug_iteration_num_topic_arg = DeclareLaunchArgument(
        'output_iteration_num_topic',
        default_value='/localization/ndt_localizer/debug/iteration_num',
        description='Output iteration number topic name for NDT localizer'
    )

    output_debug_monte_carlo_initial_pose_marker_topic_arg = DeclareLaunchArgument(
        'output_monte_carlo_initial_pose_marker_topic',
        default_value='/localization/ndt_localizer/debug/monte_carlo_initial_pose_marker',
        description='Output Monte Carlo initial pose marker topic name for NDT localizer'
    )

    output_debug_multi_initial_pose_topic_arg = DeclareLaunchArgument(
        'output_multi_initial_pose_topic',
        default_value='/localization/ndt_localizer/debug/multi_initial_pose',
        description='Output multi initial pose topic name for NDT localizer'
    )

    output_debug_multi_ndt_pose_topic_arg = DeclareLaunchArgument(
        'output_multi_ndt_pose_topic',
        default_value='/localization/ndt_localizer/debug/multi_ndt_pose',
        description='Output multi NDT pose topic name for NDT localizer'
    )

    output_debug_nearest_voxel_transformation_likelihood_topic_arg = DeclareLaunchArgument(
        'output_nearest_voxel_transformation_likelihood_topic',
        default_value='/localization/ndt_localizer/debug/nearest_voxel_transformation_likelihood',
        description='Output nearest voxel transformation likelihood topic name for NDT localizer'
    )

    output_debug_no_ground_nearest_voxel_transformation_likelihood_topic_arg = DeclareLaunchArgument(
        'output_no_ground_nearest_voxel_transformation_likelihood_topic',
        default_value='/localization/ndt_localizer/debug/no_ground_nearest_voxel_transformation_likelihood',
        description='Output no ground nearest voxel transformation likelihood topic name for NDT localizer'
    )

    output_debug_no_ground_transform_probability_topic_arg = DeclareLaunchArgument(
        'output_no_ground_transform_probability_topic',
        default_value='/localization/ndt_localizer/debug/no_ground_transform_probability',
        description='Output no ground transform probability topic name for NDT localizer'
    )

    output_debug_points_aligned_topic_arg = DeclareLaunchArgument(
        'output_points_aligned_topic',
        default_value='/localization/ndt_localizer/debug/points_aligned',
        description='Output points aligned topic name for NDT localizer'
    )

    output_debug_points_aligned_no_ground_topic_arg = DeclareLaunchArgument(
        'output_points_aligned_no_ground_topic',
        default_value='/localization/ndt_localizer/debug/points_aligned_no_ground',
        description='Output points aligned no ground topic name for NDT localizer'
    )

    output_debug_transform_probability_topic_arg = DeclareLaunchArgument(
        'output_transform_probability_topic',
        default_value='/localization/ndt_localizer/debug/transform_probability',
        description='Output transform probability topic name for NDT localizer'
    )

    ndt_localizer_node = Node(
        package = 'ndt_scan_matcher',
        executable = 'ndt_scan_matcher_node',
        output = 'both',
        parameters = [ndt_scan_matcher_param_path],
        remappings = [
            ('points_raw', LaunchConfiguration('input_pointcloud_topic')),
            ('ekf_pose_with_covariance', LaunchConfiguration('output_ekf_pose_with_covariance_topic')),
            ('regularization_pose_with_covariance', LaunchConfiguration('input_gnss_pose_with_cov_topic')),
            ('trigger_node_srv', LaunchConfiguration('input_ndt_trigger_node_service_topic')),
            ('ndt_align_srv', LaunchConfiguration('input_ndt_align_service_topic')),
            ('ndt_pose', LaunchConfiguration('output_ndt_pose_topic')),
            ('ndt_pose_with_covariance', LaunchConfiguration('input_ndt_pose_with_covariance_topic')),
            ('initial_pose_with_covariance', LaunchConfiguration('output_ndt_initial_pose_topic')),
            ('ndt_marker', LaunchConfiguration('output_ndt_marker_topic')),
            ('pcd_loader_service', LaunchConfiguration('input_client_map_loader_service_topic')),
            ('debug/loaded_pointcloud_map', LaunchConfiguration('output_debug_loaded_map_topic')),
            ('exe_time_ms', LaunchConfiguration('output_exe_time_topic')),
            ('initial_to_result_distance', LaunchConfiguration('output_initial_to_result_distance_topic')),
            ('initial_to_result_distance_new', LaunchConfiguration('output_initial_to_result_distance_new_topic')),
            ('initial_to_result_distance_old', LaunchConfiguration('output_initial_to_result_distance_old_topic')),
            ('initial_to_result_relative_pose', LaunchConfiguration('output_initial_to_result_relative_pose_topic')),
            ('iteration_num', LaunchConfiguration('output_iteration_num_topic')),
            ('monte_carlo_initial_pose_marker', LaunchConfiguration('output_monte_carlo_initial_pose_marker_topic')),
            ('multi_initial_pose', LaunchConfiguration('output_multi_initial_pose_topic')),
            ('multi_ndt_pose', LaunchConfiguration('output_multi_ndt_pose_topic')),
            ('nearest_voxel_transformation_likelihood', LaunchConfiguration('output_nearest_voxel_transformation_likelihood_topic')),
            ('no_ground_nearest_voxel_transformation_likelihood', LaunchConfiguration('output_no_ground_nearest_voxel_transformation_likelihood_topic')),
            ('no_ground_transform_probability', LaunchConfiguration('output_no_ground_transform_probability_topic')),
            ('points_aligned', LaunchConfiguration('output_points_aligned_topic')),
            ('points_aligned_no_ground', LaunchConfiguration('output_points_aligned_no_ground_topic')),
            ('transform_probability', LaunchConfiguration('output_transform_probability_topic'))
        ])


    pose_initializer_param_path = os.path.join(
        get_package_share_directory('gae_launch'),
        'config',
        'localization',
        'pose_initializer.param.yaml'
    )

    input_pointcloud_map_topic_arg = DeclareLaunchArgument(
        'input_pointcloud_map_topic',
        default_value='/map/pointcloud_map',
        description='Input pointcloud map topic name for pose initializer'
    )

    input_vector_map_topic_arg = DeclareLaunchArgument(
        'input_vector_map_topic',
        default_value='/map/lanelet2_map',
        description='Input vector map topic name for pose initializer'
    )

    pose_initializer_node = Node(
        package = 'pose_initializer',
        executable = 'pose_initializer_node',
        output = 'both',
        parameters = [pose_initializer_param_path],
        remappings = [
            ('ndt_align', LaunchConfiguration('input_ndt_align_service_topic')),
            ('gnss_pose_cov', LaunchConfiguration('input_gnss_pose_with_cov_topic')),
            ('pose_reset', 'initialpose3d'),
            ('ekf_trigger_node', LaunchConfiguration('input_trigger_node_service_topic')),
            ('ndt_trigger_node', LaunchConfiguration('input_ndt_trigger_node_service_topic')),
            ('~/pointcloud_map', LaunchConfiguration('input_pointcloud_map_topic')),
            ('~/vector_map', LaunchConfiguration('input_vector_map_topic')),
        ])

    return LaunchDescription([
        input_ndt_pose_with_cov_topic_arg,
        input_trigger_node_service_topic_arg,
        input_ndt_pose_with_covariance_topic_arg,
        input_twist_with_covariance_topic_arg,
        output_ekf_odom_topic_arg,
        output_ekf_pose_topic_arg,
        output_ekf_pose_with_cov_topic_arg,
        output_ekf_biased_pose_topic_arg,
        output_ekf_biased_pose_with_cov_topic_arg,
        output_ekf_twist_topic_arg,
        output_ekf_twist_with_cov_topic_arg,
        output_ekf_estimated_yaw_bias_topic_arg,
        ekf_localizer_node,
        input_pointcloud_topic_arg,
        input_gnss_pose_with_cov_topic_arg,
        input_ndt_trigger_node_service_topic_arg,
        input_ndt_align_service_topic_arg,
        output_ndt_pose_topic_arg,
        output_ndt_initial_pose_topic_arg,
        output_ndt_marker_topic_arg,
        input_client_map_loader_service_topic_arg,
        output_debug_loaded_map_topic_arg,
        output_debug_exe_time_topic_arg,
        output_debug_initial_to_result_distance_topic_arg,
        output_debug_initial_to_result_distance_new_topic_arg,
        output_debug_initial_to_result_distance_old_topic_arg,
        output_debug_initial_to_result_relative_pose_topic_arg,
        output_debug_iteration_num_topic_arg,
        output_debug_monte_carlo_initial_pose_marker_topic_arg,
        output_debug_multi_initial_pose_topic_arg,
        output_debug_multi_ndt_pose_topic_arg,
        output_debug_nearest_voxel_transformation_likelihood_topic_arg,
        output_debug_no_ground_nearest_voxel_transformation_likelihood_topic_arg,
        output_debug_no_ground_transform_probability_topic_arg,
        output_debug_points_aligned_topic_arg,
        output_debug_points_aligned_no_ground_topic_arg,
        output_debug_transform_probability_topic_arg,
        ndt_localizer_node,
        input_pointcloud_map_topic_arg,
        input_vector_map_topic_arg,
        pose_initializer_node
    ])
