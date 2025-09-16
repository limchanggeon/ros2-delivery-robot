#!/usr/bin/env python3
"""
완전한 GPS 기반 자율주행 시스템 런치 파일
ROS2에서 정적 지도와 GPS를 이용한 지리 참조 기반 자율 주행 시스템

이 런치 파일은 다음 구성 요소들을 실행합니다:
1. 지역 EKF (odom -> base_link 변환)
2. 전역 EKF (map -> odom 변환) 
3. navsat_transform_node (GPS -> ROS 좌표 변환)
4. map_server (정적 지도 서비스)
5. GPS 지도 보정 노드 (선택사항)

사용법:
ros2 launch capston_project gps_autonomous_system.launch.py map_file:=/path/to/your/map.yaml

필수 사전 작업:
1. 지도 파일(.yaml, .pgm) 준비
2. GPS 수신기 연결 및 /gps/fix 토픽 확인
3. IMU 연결 및 /imu/data 토픽 확인
4. 휠 오도메트리 /odom 토픽 확인
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    GroupAction, 
    IncludeLaunchDescription,
    LogInfo,
    ExecuteProcess
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # =========================
    # 런치 인수 정의
    # =========================
    
    map_file_arg = DeclareLaunchArgument(
        'map_file',
        description='지도 YAML 파일의 절대 경로 (필수)',
        default_value=''
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='시뮬레이션 시간 사용 여부'
    )
    
    enable_calibration_arg = DeclareLaunchArgument(
        'enable_calibration',
        default_value='true',
        description='GPS 지도 보정 노드 실행 여부'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='노드들의 네임스페이스'
    )
    
    # =========================
    # 설정 파일 경로
    # =========================
    
    config_dir = '/Users/limchang-geon/Desktop/capston_project/config'
    
    ekf_local_config = os.path.join(config_dir, 'ekf_local.yaml')
    ekf_global_config = os.path.join(config_dir, 'ekf_global.yaml') 
    navsat_config = os.path.join(config_dir, 'navsat_transform.yaml')
    calibration_config = os.path.join(config_dir, 'gps_calibration_params.yaml')
    
    # =========================
    # 공통 파라미터
    # =========================
    
    common_params = {
        'use_sim_time': LaunchConfiguration('use_sim_time')
    }
    
    # =========================
    # 노드 정의
    # =========================
    
    # 1. 지역 EKF (연속 센서 융합 -> odom -> base_link)
    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_local',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            ParameterFile(ekf_local_config, allow_substs=True),
            common_params
        ],
        remappings=[
            ('odometry/filtered', 'odometry/local'),
        ]
    )
    
    # 2. navsat_transform_node (GPS -> ROS 좌표 변환)
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            ParameterFile(navsat_config, allow_substs=True),
            common_params
        ],
        remappings=[
            ('imu/data', 'imu/data'),
            ('gps/fix', 'gps/fix'),
            ('odometry/filtered', 'odometry/local'),
            ('odometry/gps', 'odometry/gps'),
            ('gps/filtered', 'gps/filtered')
        ]
    )
    
    # 3. 전역 EKF (GPS + 지역 오도메트리 융합 -> map -> odom)
    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node', 
        name='ekf_filter_global',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            ParameterFile(ekf_global_config, allow_substs=True),
            common_params
        ],
        remappings=[
            ('odometry/filtered', 'odometry/global'),
        ]
    )
    
    # 4. Map Server (정적 지도 서비스)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map_file'),
            **common_params
        }],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('map_file'), "' != ''"]) 
        )
    )
    
    # 5. Map Server Lifecycle Manager
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[{
            'node_names': ['map_server'],
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True
        }],
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('map_file'), "' != ''"]) 
        )
    )
    
    # 6. GPS 지도 보정 노드 (선택사항)
    gps_calibrator_node = Node(
        package='capston_project',
        executable='map_ros.py',
        name='gps_map_calibrator',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[
            ParameterFile(calibration_config, allow_substs=True),
            common_params
        ],
        condition=IfCondition(LaunchConfiguration('enable_calibration'))
    )
    
    # =========================
    # 시작 메시지
    # =========================
    
    start_message = LogInfo(
        msg=[
            '\\n' + '='*80 + '\\n',
            '🚀 GPS 기반 자율주행 시스템 시작\\n',
            '='*80 + '\\n',
            '📋 시스템 구성요소:\\n',
            '   • 지역 EKF: 휠 오도메트리 + IMU 융합 (odom -> base_link)\\n',
            '   • 전역 EKF: 지역 오도메트리 + GPS 융합 (map -> odom)\\n', 
            '   • NavSat Transform: GPS -> ROS 좌표 변환\\n',
            '   • Map Server: 정적 지도 서비스\\n',
            '   • GPS 보정 노드: 지도-GPS 정렬 (선택사항)\\n',
            '\\n📡 필수 토픽 확인:\\n',
            '   • /gps/fix (sensor_msgs/NavSatFix)\\n',
            '   • /imu/data (sensor_msgs/Imu)\\n', 
            '   • /odom (nav_msgs/Odometry)\\n',
            '\\n🗺️ 지도 보정 방법:\\n',
            '   1. RViz 실행 및 Fixed Frame을 \"map\"으로 설정\\n',
            '   2. GPS 신호 수신 대기\\n',
            '   3. \"Publish Point\" 도구로 지도 위 한 점 클릭\\n',
            '   4. UTM -> MAP 변환 자동 설정 완료!\\n',
            '='*80 + '\\n'
        ]
    )
    
    # =========================
    # 진단 도구 (선택사항)  
    # =========================
    
    # TF 트리 모니터링
    tf_monitor = ExecuteProcess(
        cmd=['ros2', 'run', 'tf2_tools', 'view_frames'],
        output='screen',
        condition=IfCondition('false')  # 기본적으로 비활성화
    )
    
    # =========================
    # 런치 구성
    # =========================
    
    # 네임스페이스가 있는 경우 그룹으로 묶기
    system_group = GroupAction([
        ekf_local_node,
        navsat_transform_node, 
        ekf_global_node,
        map_server_node,
        lifecycle_manager_node,
        gps_calibrator_node,
    ])
    
    return LaunchDescription([
        # 런치 인수들
        map_file_arg,
        use_sim_time_arg,
        enable_calibration_arg,
        namespace_arg,
        
        # 시작 메시지
        start_message,
        
        # 메인 시스템 그룹
        PushRosNamespace(LaunchConfiguration('namespace')),
        system_group,
        
        # 진단 도구 (주석 해제하여 사용)
        # tf_monitor,
    ])