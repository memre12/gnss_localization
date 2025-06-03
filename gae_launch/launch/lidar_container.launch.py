# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Launch the velodyne driver, pointcloud, and laserscan nodes in a composable container with default configuration."""

import os

import ament_index_python.packages
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    driver_share_dir = ament_index_python.packages.get_package_share_directory('gae_launch')
    driver_params_file = os.path.join(driver_share_dir, 'config', 'lidar', 'velodyne_driver.param.yaml')
    with open(driver_params_file, 'r') as f:
        driver_params = yaml.safe_load(f)['velodyne_driver_node']['ros__parameters']

    convert_share_dir = ament_index_python.packages.get_package_share_directory('gae_launch')
    convert_params_file = os.path.join(convert_share_dir, 'config','lidar', 'velodyne_convert.param.yaml')
    with open(convert_params_file, 'r') as f:
        convert_params = yaml.safe_load(f)['velodyne_convert_node']['ros__parameters']
    convert_params['calibration'] = os.path.join(ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')
                                                 , 'params', 'VLP16db.yaml')

    cropbox_filter_param_path = os.path.join(
        ament_index_python.packages.get_package_share_directory('gae_launch'),
        'config',
        'lidar',
        'cropbox.param.yaml'
    )

    output_cropbox_filtered_points_topic_arg = DeclareLaunchArgument(
        'output_cropbox_filtered_points_topic',
        default_value='/sensing/lidar_top/points_cropbox_filtered',
        description='Output cropbox filtered points topic name'
    )
    
    distorsion_corrector_param_path = os.path.join(
        ament_index_python.packages.get_package_share_directory('gae_launch'),
        'config',
        'lidar',
        'distortion_corrector_node.param.yaml'
    )
    
    input_twist_topic_arg = DeclareLaunchArgument(
        'input_twist_topic',
        default_value='/eagleye/twist_with_covariance',
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
    
    downsample_param_path = os.path.join(
        ament_index_python.packages.get_package_share_directory('gae_launch'),
        'config',
        'lidar',
        'downsample.param.yaml'
    )
    
    output_downsampled_points_topic_arg = DeclareLaunchArgument(
        'output_downsampled_points_topic',
        default_value='/sensing/lidar_top/points_downsampled',
        description='Output downsampled points topic name'
    )

    default_ground_segmentation_param_path = os.path.join(
        ament_index_python.packages.get_package_share_directory(
            "ground_segmentation"
        ),
        "config/ground_segmentation.param.yaml",
    )

    default_object_clustering_param_path = os.path.join(
        ament_index_python.packages.get_package_share_directory(
            "object_clustering"
        ),
        "config/object_clustering.param.yaml",
    )
    
    container = ComposableNodeContainer(
            name='velodyne_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='velodyne_driver',
                    plugin='velodyne_driver::VelodyneDriver',
                    name='velodyne_driver_node',
                    parameters=[driver_params]),
                ComposableNode(
                    package='velodyne_pointcloud',
                    plugin='velodyne_pointcloud::Convert',
                    name='velodyne_convert_node',
                    parameters=[convert_params]),
                ComposableNode(
                    package='pointcloud_preprocessor',
                    plugin='pointcloud_preprocessor::CropBoxFilterComponent',
                    name='cropbox_filter',
                    parameters=[cropbox_filter_param_path],

                    remappings=[
                        ('input', '/velodyne_points'),
                        ('output', LaunchConfiguration('output_cropbox_filtered_points_topic'))
                        ]),
                ComposableNode(
                    package='pointcloud_preprocessor',
                    plugin='pointcloud_preprocessor::DistortionCorrectorComponent',
                    name='distortion_corrector',
                    parameters=[distorsion_corrector_param_path],
                    
                    remappings=[
                        ('~/input/pointcloud', LaunchConfiguration('output_cropbox_filtered_points_topic')),
                        ('~/input/twist', LaunchConfiguration('input_twist_topic')),
                        ('~/input/imu', LaunchConfiguration('input_imu_topic')),
                        ('~/output/pointcloud', LaunchConfiguration('output_corrected_points_topic'))
                    ]),
                ComposableNode(
                    package='pointcloud_preprocessor',
                    plugin='pointcloud_preprocessor::VoxelGridDownsampleFilterComponent',
                    name='voxel_grid_downsample_filter',
                    parameters=[downsample_param_path],
                    remappings=[
                        ('input', LaunchConfiguration('output_corrected_points_topic')),
                        ('output', LaunchConfiguration('output_downsampled_points_topic'))
                    ]),
    ])
    
    return LaunchDescription([output_cropbox_filtered_points_topic_arg,
                              input_twist_topic_arg,
                              input_imu_topic_arg,
                              output_corrected_points_topic_arg,
                              output_downsampled_points_topic_arg,
                              container
                             ])
