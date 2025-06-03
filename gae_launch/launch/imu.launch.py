import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    imu_param_path = os.path.join(
        get_package_share_directory('sbg_driver'),
        'config',
        'sbg_device_uart_default.yaml'
    )

    imu_driver_node = Node(
        package = 'sbg_driver',
        executable = 'sbg_device',
        output = 'both',
        parameters = [imu_param_path]
    )

    
    imu_corrector_param_path = os.path.join(
        get_package_share_directory('gae_launch'),
        'config',
        'imu',
        'imu_corrector.param.yaml'
    )

    input_imu_topic_arg = DeclareLaunchArgument(
        'input_imu_topic',
        default_value='/sbg/ros/imu/data',
        description='Input IMU topic name'
    )

    output_corrected_imu_topic_arg = DeclareLaunchArgument(
        'output_corrected_imu_topic',
        default_value='/sensing/imu/imu_corrected',
        description='Output corrected IMU topic name'
    )

    imu_corrector_node = Node(
        package = 'imu_corrector',
        executable = 'imu_corrector_node',
        output = 'both',
        parameters = [imu_corrector_param_path],
        remappings = [
            ('input', LaunchConfiguration('input_imu_topic')),
            ('output', LaunchConfiguration('output_corrected_imu_topic'))
        ]
    )
        

    gnss_poser_param_path = os.path.join(
        get_package_share_directory('gae_launch'),
        'config',
        'imu',
        'gnss_poser.param.yaml'
    )
    
    input_topic_fix_arg = DeclareLaunchArgument(
        'input_topic_fix',
        default_value='/sbg/ekf_navsatfix',
        description='Input GNSS fix topic name'
    )

    input_topic_orientation_arg = DeclareLaunchArgument(
        'input_topic_orientation',
        default_value='autoware/sbg/ins',
        description='Input orientation topic name'
    )

    output_topic_gnss_pose_arg = DeclareLaunchArgument(
        'output_topic_gnss_pose',
        default_value='/sensing/gnss/gnss_pose',
        description='Output GNSS pose topic name'
    )

    output_topic_gnss_pose_with_cov_arg = DeclareLaunchArgument(
        'output_topic_gnss_pose_with_cov',
        default_value='/sensing/gnss/gnss_pose_with_cov',
        description='Output GNSS pose with cov topic name'
    )

    output_topic_gnss_fixed_arg = DeclareLaunchArgument(
        'output_topic_gnss_fixed',
        default_value='/sensing/gnss/gnss_fixed',
        description='Output GNSS fixed topic name'
    )

    gnss_poser_node = Node(
        package = 'gnss_poser',
        executable = 'gnss_poser',
        output = 'both',
        parameters = [gnss_poser_param_path],
        remappings = [
            ('fix', LaunchConfiguration('input_topic_fix')),
            ('autoware_orientation', LaunchConfiguration('input_topic_orientation')),
            ('gnss_pose', LaunchConfiguration('output_topic_gnss_pose')),
            ('gnss_pose_cov', LaunchConfiguration('output_topic_gnss_pose_with_cov')),
            ('gnss_fixed', LaunchConfiguration('output_topic_gnss_fixed'))
        ]
    )

    return LaunchDescription([
        imu_driver_node,
        input_imu_topic_arg,
        output_corrected_imu_topic_arg,
        imu_corrector_node,
        input_topic_fix_arg,
        input_topic_orientation_arg,
        output_topic_gnss_pose_arg,
        output_topic_gnss_pose_with_cov_arg,
        output_topic_gnss_fixed_arg,
        gnss_poser_node
    ])