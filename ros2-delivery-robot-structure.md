# ROS 2 무인 배달 로봇 프로젝트 구조

## 워크스페이스 디렉토리 구조

```
ros2_ws/
├── src/
│   ├── delivery_robot_description/     # 로봇 모델 및 URDF 파일
│   │   ├── launch/
│   │   │   └── robot_description.launch.py
│   │   ├── urdf/
│   │   │   ├── delivery_robot.urdf.xacro
│   │   │   └── materials.xacro
│   │   ├── meshes/
│   │   │   └── robot_chassis.stl
│   │   ├── config/
│   │   │   └── joint_limits.yaml
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   │
│   ├── delivery_robot_navigation/      # 내비게이션 관련 패키지
│   │   ├── config/
│   │   │   ├── nav2_params.yaml
│   │   │   ├── ekf.yaml
│   │   │   └── costmap_plugins.yaml
│   │   ├── launch/
│   │   │   ├── navigation_launch.py
│   │   │   └── localization_launch.py
│   │   ├── maps/
│   │   │   ├── warehouse.yaml
│   │   │   └── warehouse.pgm
│   │   ├── params/
│   │   │   └── planner_server_params.yaml
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   │
│   ├── delivery_robot_perception/      # 인식 시스템 (YOLOv8, 카메라)
│   │   ├── delivery_robot_perception/
│   │   │   ├── __init__.py
│   │   │   ├── yolo_inference_node.py
│   │   │   ├── camera_driver_node.py
│   │   │   └── yolo_costmap_plugin.cpp
│   │   ├── launch/
│   │   │   ├── perception_launch.py
│   │   │   └── camera_launch.py
│   │   ├── config/
│   │   │   ├── yolo_config.yaml
│   │   │   └── camera_params.yaml
│   │   ├── models/
│   │   │   ├── yolov8n.pt
│   │   │   └── yolov8n.onnx
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
│   │
│   ├── delivery_robot_security/        # 보안 및 QR 코드 인증
│   │   ├── delivery_robot_security/
│   │   │   ├── __init__.py
│   │   │   ├── authentication_node.py
│   │   │   ├── qr_reader_node.py
│   │   │   └── door_control_node.py
│   │   ├── launch/
│   │   │   └── security_launch.py
│   │   ├── config/
│   │   │   ├── qr_params.yaml
│   │   │   └── door_config.yaml
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
│   │
│   ├── delivery_robot_control/         # 로봇 제어 및 하드웨어 인터페이스
│   │   ├── delivery_robot_control/
│   │   │   ├── __init__.py
│   │   │   ├── hardware_interface.py
│   │   │   └── motor_controller_node.py
│   │   ├── launch/
│   │   │   └── control_launch.py
│   │   ├── config/
│   │   │   ├── ros2_control.yaml
│   │   │   └── motor_params.yaml
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
│   │
│   └── delivery_robot_mission/         # 임무 관리 및 지도 API 연동
│       ├── delivery_robot_mission/
│       │   ├── __init__.py
│       │   ├── mission_control_node.py
│       │   ├── map_api_interface.py
│       │   └── waypoint_follower.py
│       ├── launch/
│       │   └── mission_launch.py
│       ├── config/
│       │   ├── mission_params.yaml
│       │   └── api_keys.yaml
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
├── build/
├── install/
└── log/
```

## 각 패키지별 상세 기능

### 1. delivery_robot_description
- **목적**: 로봇의 물리적 모델 정의 (URDF, 센서 위치, joint 정보)
- **주요 파일**: 
  - `delivery_robot.urdf.xacro`: 로봇 모델 정의
  - `robot_description.launch.py`: 로봇 모델 발행

### 2. delivery_robot_navigation
- **목적**: Nav2 스택을 이용한 자율 주행, 경로 계획, 장애물 회피
- **주요 파일**:
  - `nav2_params.yaml`: Nav2 설정
  - `ekf.yaml`: robot_localization 설정
  - `navigation_launch.py`: 전체 내비게이션 스택 실행

### 3. delivery_robot_perception
- **목적**: YOLOv8 기반 객체 인식, 카메라 드라이버, 시각적 장애물 탐지
- **주요 파일**:
  - `yolo_inference_node.py`: YOLOv8 추론 노드
  - `yolo_costmap_plugin.cpp`: Nav2용 커스텀 코스트맵 플러그인

### 4. delivery_robot_security
- **목적**: QR 코드 인증, 도어 제어, 보안 관리
- **주요 파일**:
  - `authentication_node.py`: QR 인증 로직
  - `qr_reader_node.py`: QR 코드 탐지 및 디코딩
  - `door_control_node.py`: 물리적 도어 제어

### 5. delivery_robot_control
- **목적**: ros2_control 기반 하드웨어 인터페이스, 모터 제어
- **주요 파일**:
  - `hardware_interface.py`: 하드웨어 추상화
  - `ros2_control.yaml`: 제어 파라미터

### 6. delivery_robot_mission
- **목적**: 고수준 임무 관리, 외부 지도 API 연동, 경로 생성
- **주요 파일**:
  - `mission_control_node.py`: 임무 상태 기계
  - `map_api_interface.py`: 카카오맵 API 연동

## 주요 런치 파일 구성

### 전체 시스템 실행
```bash
ros2 launch delivery_robot_mission full_system.launch.py
```

### 개별 서브시스템 실행
```bash
# 로봇 모델 발행
ros2 launch delivery_robot_description robot_description.launch.py

# 인식 시스템
ros2 launch delivery_robot_perception perception_launch.py

# 내비게이션 스택
ros2 launch delivery_robot_navigation navigation_launch.py

# 보안 시스템
ros2 launch delivery_robot_security security_launch.py
```