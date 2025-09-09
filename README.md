# ROS 2 자율주행 배송 로봇 시스템

이 프로젝트는 ROS 2 Humble을 기반으로 한 완전 자율주행 배송 로봇 시스템입니다. YOLOv8 객체 인식, GPS/IMU 기반 위치 추정, QR 코드 인증, Nav2 네비게이션을 통합한 포괄적인 배송 솔루션을 제공합니다.

## 📋 주요 기능

- **🤖 자율 주행**: Nav2 기반 경로 계획 및 장애물 회피
- **👁️ 객체 인식**: YOLOv8을 활용한 실시간 객체 탐지
- **📍 정밀 위치 추정**: GPS + IMU 센서 융합 (EKF)
- **🔒 보안 인증**: QR 코드 기반 배송 인증 시스템
- **🗺️ 지도 API 연동**: Kakao Map API를 통한 경로 생성
- **📊 시스템 모니터링**: 실시간 상태 감시 및 로그

## 🏗️ 프로젝트 구조

```
capston_project/
├── 📁 models/                           # YOLOv8 모델 파일
│   ├── yolov8_best.pt                  # 훈련된 YOLOv8 모델
│   └── README.md                       # 모델 정보
├── 📁 src/                             # ROS 2 패키지들
│   ├── delivery_robot_description/     # 로봇 모델 정의 (URDF)
│   │   ├── urdf/
│   │   │   ├── delivery_robot.urdf.xacro
│   │   │   └── materials.xacro
│   │   └── launch/
│   │       └── robot_description.launch.py
│   ├── delivery_robot_navigation/      # 네비게이션 시스템
│   │   ├── config/
│   │   │   ├── nav2_params.yaml
│   │   │   └── ekf.yaml
│   │   ├── launch/
│   │   │   ├── navigation.launch.py
│   │   │   └── localization.launch.py
│   │   └── maps/
│   │       └── warehouse.yaml
│   ├── delivery_robot_perception/      # 객체 인식 (YOLOv8)
│   │   ├── delivery_robot_perception/
│   │   │   ├── yolo_inference_node.py
│   │   │   └── camera_driver_node.py
│   │   ├── config/
│   │   │   └── yolo_config.yaml
│   │   └── launch/
│   │       └── perception.launch.py
│   ├── delivery_robot_security/        # QR 코드 인증
│   │   ├── delivery_robot_security/
│   │   │   └── authentication_node.py
│   │   ├── config/
│   │   │   └── qr_params.yaml
│   │   └── launch/
│   │       └── security.launch.py
│   ├── delivery_robot_control/         # 하드웨어 제어
│   │   └── config/
│   │       └── ros2_control.yaml
│   └── delivery_robot_mission/         # 미션 관리
│       ├── delivery_robot_mission/
│       │   ├── mission_control_node.py
│       │   └── system_monitor_node.py
│       ├── config/
│       │   └── mission_params.yaml
│       └── launch/
│           ├── mission.launch.py
│           └── full_system_launch.py
├── 📄 build_and_run.sh                 # 빌드 및 실행 스크립트
├── 📄 test_system.sh                   # 시스템 테스트 스크립트
├── 📄 .gitignore                       # Git 무시 파일
└── 📄 README.md                        # 프로젝트 문서
```

## 📦 패키지 상세

### 1. delivery_robot_description
로봇의 물리적 모델과 시뮬레이션 환경을 정의합니다.
- **URDF/Xacro**: 로봇 구조, 센서 배치, 물리적 속성
- **Gazebo 플러그인**: 시뮬레이션 환경 지원
- **Material 정의**: 로봇 외관 및 물리적 특성

### 2. delivery_robot_navigation
자율주행을 위한 네비게이션 시스템입니다.
- **Nav2 설정**: 경로 계획, 장애물 회피, 지역 계획
- **EKF 센서 융합**: GPS, IMU, 오도메트리 데이터 통합
- **맵 관리**: SLAM 및 미리 작성된 맵 지원

### 3. delivery_robot_perception
실시간 환경 인식 시스템입니다.
- **YOLOv8 추론**: 사람, 차량, 장애물 탐지
- **카메라 드라이버**: USB/IP 카메라 지원
- **데이터 처리**: OpenCV 기반 이미지 전처리

### 4. delivery_robot_security
배송 보안 및 인증 시스템입니다.
- **QR 코드 스캐닝**: zbar 라이브러리 활용
- **서버 통신**: 암호화된 인증 프로토콜
- **하드웨어 제어**: 서보 모터 기반 도어 제어

### 5. delivery_robot_control
로봇 하드웨어 제어 인터페이스입니다.
- **ros2_control**: 모터 드라이버, 센서 인터페이스
- **차동 구동**: differential drive 제어
- **GPIO 제어**: 조명, 부저, 센서 제어

### 6. delivery_robot_mission
전체 미션 관리 및 외부 API 연동입니다.
- **상태 머신**: 미션 단계별 제어
- **Kakao Map API**: 경로 생성 및 최적화
- **시스템 모니터링**: 성능 및 상태 추적

## 🚀 설치 및 실행

### 사전 요구사항

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Python 3.8+
- CUDA (GPU 사용 시, 선택사항)

### 자동 설치 및 실행

```bash
# 프로젝트 디렉토리로 이동
cd capston_project

# 자동 빌드 및 실행
chmod +x build_and_run.sh
./build_and_run.sh
```

### 수동 설치

1. **의존성 설치**
```bash
# ROS 2 의존성
sudo apt update
sudo apt install -y python3-pip python3-colcon-common-extensions
sudo apt install -y ros-humble-nav2-bringup ros-humble-robot-localization
sudo apt install -y ros-humble-vision-msgs ros-humble-cv-bridge
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers

# Python 의존성
pip3 install ultralytics torch opencv-python numpy geopy requests cryptography
```

2. **빌드**
```bash
colcon build --symlink-install
source install/setup.bash
```

3. **실행**
```bash
# 전체 시스템 실행
ros2 launch delivery_robot_mission full_system_launch.py

# 또는 개별 패키지 실행
ros2 launch delivery_robot_perception perception.launch.py
ros2 launch delivery_robot_navigation navigation.launch.py
```

## 🧪 테스트

시스템 테스트를 위한 스크립트가 제공됩니다:

```bash
chmod +x test_system.sh
./test_system.sh
```

개별 노드 테스트:
```bash
# YOLOv8 인식 테스트
ros2 run delivery_robot_perception yolo_inference_node

# 네비게이션 테스트
ros2 launch delivery_robot_navigation navigation.launch.py

# QR 코드 인증 테스트
ros2 run delivery_robot_security authentication_node
```

## 📊 모델 정보

### YOLOv8 모델 (models/yolov8_best.pt)

- **모델 타입**: YOLOv8 (You Only Look Once v8)
- **용도**: 실시간 객체 탐지 및 인식
- **탐지 객체**: 사람, 차량, 장애물, 신호등 등
- **성능**: GPU/CPU 환경 모두 지원

모델 세부 정보는 `models/README.md`를 참조하세요.

## 🔧 설정

주요 설정 파일들:

- `src/delivery_robot_navigation/config/nav2_params.yaml`: 네비게이션 파라미터
- `src/delivery_robot_navigation/config/ekf.yaml`: 센서 융합 설정
- `src/delivery_robot_perception/config/yolo_config.yaml`: YOLO 추론 설정
- `src/delivery_robot_security/config/qr_params.yaml`: QR 인증 설정

## 🚨 문제 해결

### 일반적인 문제들

1. **YOLO 모델 로드 실패**
   - `models/yolov8_best.pt` 파일 존재 확인
   - PyTorch 및 Ultralytics 설치 확인

2. **네비게이션 오류**
   - TF tree 연결 상태 확인: `ros2 run tf2_tools view_frames`
   - 센서 토픽 발행 확인: `ros2 topic list`

3. **빌드 오류**
   - 의존성 재설치: `rosdep install --from-paths src --ignore-src -r -y`
   - 캐시 정리: `rm -rf build install log`

## 📜 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다.

## 👥 기여

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

## 📞 연락처

프로젝트 관련 문의사항이 있으시면 이슈를 생성해주세요.

---

⭐ 이 프로젝트가 도움이 되었다면 스타를 눌러주세요!