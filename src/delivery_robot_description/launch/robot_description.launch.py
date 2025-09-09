#!/usr/bin/env python3
"""
로봇 모델 런치 파일
URDF를 로드하고 robot_state_publisher를 실행하는 런치 파일

작성자: 배달로봇팀
날짜: 2025-09-09
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """로봇 모델 런치 파일"""
    
    # 런치 인수 선언
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='시뮬레이션 시간 사용 여부'
    )
    
    declare_use_gui = DeclareLaunchArgument(
        'use_gui',
        default_value='false',
        description='joint_state_publisher_gui 사용 여부'
    )
    
    # 런치 설정 변수
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gui = LaunchConfiguration('use_gui')
    
    # URDF 파일 경로
    urdf_file = PathJoinSubstitution([
        FindPackageShare('delivery_robot_description'),
        'urdf',
        'delivery_robot.urdf.xacro'
    ])
    
    # robot_description 파라미터 생성
    robot_description = Command(['xacro ', urdf_file])
    
    # robot_state_publisher 노드
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }]
    )
    
    # joint_state_publisher 노드
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        condition=UnlessCondition(use_gui)
    )
    
    # joint_state_publisher_gui 노드
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(use_gui)
    )
    
    return LaunchDescription([
        # 런치 인수
        declare_use_sim_time,
        declare_use_gui,
        
        # 노드들
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
    ])