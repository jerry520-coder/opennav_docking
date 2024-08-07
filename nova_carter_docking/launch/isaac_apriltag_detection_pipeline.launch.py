#!/usr/bin/env python3

# Copyright (c) 2024 Open Navigation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node

def generate_launch_description():

    # 创建一个可组合节点，用于图像矫正
    rectify_node = ComposableNode(
        name='rectify_node',
        package='isaac_ros_image_proc',
        plugin='nvidia::isaac_ros::image_proc::RectifyNode',
        parameters=[{
            'output_width': 1920,
            'output_height': 1200,
        }],
        remappings=[
            ('image_raw', '/hawk_front/left/image_raw'),# 原始图像话题
            ('camera_info', '/hawk_front/left/camerainfo'),# 相机信息话题
            ('image_rect', '/hawk_front/left/image_rect'),# 矫正后的图像话题
            ('camera_info_rect', '/hawk_front/left/camera_info_rect')# 矫正后的相机信息话题
        ]
    )

    # 创建一个可组合节点，用于AprilTag检测
    apriltag_node = ComposableNode(
        package='isaac_ros_apriltag',
        plugin='nvidia::isaac_ros::apriltag::AprilTagNode',
        name='apriltag',
        remappings=[
            ('image', '/hawk_front/left/image_rect'),        # /owl_front/left/image_raw # 矫正后的图像话题
            ('camera_info', '/hawk_front/left/camera_info_rect')  # /owl_front/left/camerainfo # 矫正后的相机信息话题
        ],
        parameters=[{'size': 0.1524,  # 6 inches # AprilTag的大小（6英寸）
                     'max_tags': 4, # 最大检测标签数量
                     'tile_size': 4}]) # AprilTag检测算法的tile大小

    # 创建一个组合节点容器，用于运行可组合节点
    apriltag_container = ComposableNodeContainer(
        package='rclcpp_components',
        name='apriltag_container',
        namespace='',
        executable='component_container_mt',
        composable_node_descriptions=[
            rectify_node,
            apriltag_node,
        ],
        output='screen'
    )

# 创建一个节点，用于发布对接位姿
    dock_pose_publisher = Node(
        package='nova_carter_docking',
        executable='dock_pose_publisher',
        name='dock_pose_publisher',
        parameters=[{'use_first_detection': True}],
    )

    return launch.LaunchDescription([apriltag_container, dock_pose_publisher])
