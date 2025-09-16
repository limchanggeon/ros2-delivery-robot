#!/usr/bin/env python3
"""
GPS 기반 지도 보정을 위한 런치 파일
사용법: ros2 launch capston_project gps_calibration_launch.py

이 런치 파일은 다음을 실행합니다:
1. map_server - 정적 지도 로딩
2. gps_map_calibrator - GPS 지도 보정 노드
3. RViz2 - 시각화 및 클릭 포인트 선택

런치 후 RViz에서 "Publish Point" 도구를 사용하여 지도 위 한 점을 클릭하면
해당 지점과 현재 GPS 위치를 기반으로 UTM -> MAP 변환이 자동 설정됩니다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 런치 인수 정의
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        default_value='',
        description='지도 YAML 파일의 전체 경로 (예: /path/to/map.yaml)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='시뮬레이션 시간 사용 여부'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value='',
        description='RViz 설정 파일 경로 (선택사항)'
    )
    
    # 파라미터 경로 설정
    calibration_params_file = os.path.join(
        get_package_share_directory('capston_project') if os.path.exists('/opt/ros/humble/share/capston_project') 
        else '/Users/limchang-geon/Desktop/capston_project',
        'config',
        'gps_calibration_params.yaml'
    )
    
    # 1. Map Server 노드
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        condition=lambda context: context.launch_configurations['map_file'] != ''
    )
    
    # 2. GPS 지도 보정 노드
    gps_calibrator_node = Node(
        package='capston_project',  # 패키지명은 실제 프로젝트에 맞게 수정 필요
        executable='gps_map_calibrator',
        name='gps_map_calibrator',
        output='screen',
        parameters=[calibration_params_file, {
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # 3. RViz2 실행
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')] 
                  if LaunchConfiguration('rviz_config') != '' else [],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # 시작 안내 메시지
    start_message = LogInfo(
        msg=[
            '\n' + '='*60 + '\n',
            '🚀 GPS 지도 보정 시스템 시작됨\n',
            '='*60 + '\n',
            '📋 사용 방법:\n',
            '1. RViz에서 Fixed Frame을 "map"으로 설정\n',
            '2. Map display 추가하여 지도 확인\n',
            '3. GPS 신호가 수신될 때까지 대기\n',
            '4. "Publish Point" 도구로 지도 위 한 점 클릭\n',
            '5. UTM -> MAP 변환이 자동으로 설정됩니다!\n',
            '='*60 + '\n'
        ]
    )
    
    return LaunchDescription([
        # 런치 인수
        map_file_arg,
        use_sim_time_arg,
        rviz_config_arg,
        
        # 시작 메시지
        start_message,
        
        # 노드들
        map_server_node,
        gps_calibrator_node,
        rviz_node,
    ])