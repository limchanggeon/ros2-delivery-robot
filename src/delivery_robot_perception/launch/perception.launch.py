#!/usr/bin/env python3
"""
인식 시스템 런치 파일
YOLOv8 추론 노드와 카메라 드라이버를 실행

작성자: 배달로봇팀
날짜: 2025-09-09
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """인식 시스템 런치 파일"""
    
    # 런치 인수 선언
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='시뮬레이션 시간 사용 여부'
    )
    
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('delivery_robot_perception'),
            'config',
            'yolo_config.yaml'
        ]),
        description='YOLO 설정 파일 경로'
    )
    
    # 런치 설정 변수
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')
    
    # 카메라 드라이버 노드
    camera_driver_node = Node(
        package='delivery_robot_perception',
        executable='camera_driver_node',
        name='camera_driver',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # YOLOv8 추론 노드
    yolo_inference_node = Node(
        package='delivery_robot_perception',
        executable='yolo_inference_node',
        name='yolo_inference',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    return LaunchDescription([
        # 런치 인수
        declare_use_sim_time,
        declare_config_file,
        
        # 노드들
        camera_driver_node,
        yolo_inference_node,
    ])