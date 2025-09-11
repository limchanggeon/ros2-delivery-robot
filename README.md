# 🚀 ROS2 자율주행 배송 로봇 + NARCHON 통합 관제 시스템

> **완전한 로봇 플릿 관리 솔루션** - ROS2 기반 자율주행 배송 로봇에 실시간 웹 관제 시스템을 통합한 포괄적인 솔루션

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.8+-green)](https://www.python.org/)
[![FastAPI](https://img.shields.io/badge/FastAPI-0.100+-red)](https://fastapi.tiangolo.com/)
[![License](https://img.shields.io/badge/License-MIT-yellow)](LICENSE)

## � 프로젝트 개요

본 프로젝트는 **두 가지 핵심 시스템**을 통합한 완전한 배송 로봇 솔루션입니다:

### 🤖 **ROS2 자율주행 배송 로봇** (기본 시스템)
- YOLOv8 기반 실시간 객체 인식
- GPS/IMU 센서 융합을 통한 정밀 위치 추정
- Nav2 기반 자율 네비게이션 및 장애물 회피
- QR 코드 기반 보안 인증 시스템
- Kakao Map API 연동 경로 생성

### 🎮 **NARCHON 통합 관제 시스템** (신규 추가)
- **실시간 웹 대시보드**: 직관적인 로봇 플릿 모니터링
- **다중 로봇 관리**: 여러 로봇 동시 제어 및 상태 추적
- **실시간 텔레메트리**: 배터리, 위치, 속도, 시스템 상태 모니터링
- **2D 맵 시각화**: Leaflet.js 기반 실시간 로봇 위치 추적
- **비디오 스트리밍**: WebRTC 지원 실시간 카메라 피드
- **미션 관리**: 복잡한 배송 워크플로우 생성 및 실행
- **원격 제어**: 웹 인터페이스를 통한 로봇 명령 전송

## 🏗️ 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────────┐
│                    🌐 Web Dashboard (Frontend)                    │
│                      http://localhost:8000                       │
└─────────────────────┬───────────────────────────────────────────┘
                      │ WebSocket/REST API
┌─────────────────────▼───────────────────────────────────────────┐
│                  ⚡ FastAPI Backend Server                      │
│              (WebSocket + REST + Database)                      │
└─────────────────────┬───────────────────────────────────────────┘
                      │ ROS2 Bridge
┌─────────────────────▼───────────────────────────────────────────┐
│                   🤖 ROS2 Robot System                         │
│  ┌─────────────┬─────────────┬─────────────┬─────────────────┐  │
│  │ Perception  │ Navigation  │   Control   │    Mission      │  │
│  │   (YOLO)    │   (Nav2)    │  (Hardware) │  (Workflow)     │  │
│  └─────────────┴─────────────┴─────────────┴─────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

## 📂 프로젝트 구조

```
ros2-delivery-robot/
├── 📁 Integrated Control System/       # 🆕 NARCHON 통합 관제 시스템
│   ├── 🔧 FastAPI.py                   # 백엔드 API 서버 (450+ lines)
│   ├── 🌐 frontend/                    # 반응형 웹 대시보드
│   │   ├── index.html                  # 메인 UI (Chart.js + Leaflet.js)
│   │   └── script.js                   # 클라이언트 로직 + WebSocket
│   ├── 📊 status_publisher_node.py     # 로봇 텔레메트리 수집 노드
│   ├── 🌉 web_bridge_node.py           # ROS2 ↔ 웹 통신 브리지
│   ├── 🚀 launch/                      # 통합 시스템 런치 파일
│   ├── ⚡ start_system.sh              # 원클릭 시스템 시작
│   ├── 📈 check_status.sh              # 시스템 상태 모니터링
│   ├── 🗃️ robot_control.db             # SQLite 데이터베이스
│   └── 📖 README.md                    # 상세 사용 가이드
├── 📁 src/                             # ROS2 패키지들
│   ├── 📁 delivery_robot_description/  # 로봇 모델 정의 (URDF)
│   ├── 📁 delivery_robot_control/      # 하드웨어 제어 인터페이스
│   ├── 📁 delivery_robot_navigation/   # 자율 네비게이션 (Nav2)
│   ├── 📁 delivery_robot_perception/   # 인식 시스템 (YOLOv8)
│   ├── 📁 delivery_robot_mission/      # 미션 관리 및 실행
│   └── 📁 delivery_robot_security/     # 보안 및 인증 (QR)
├── 📁 models/                          # AI 모델 파일
│   ├── yolov8_best.pt                  # 훈련된 YOLOv8 모델
│   └── README.md                       # 모델 상세 정보
├── 📁 scripts/                         # 설치 및 실행 스크립트
│   ├── build_and_run.sh                # 통합 빌드 및 실행
│   ├── build_and_run_jetson.sh         # Jetson 최적화 빌드
│   ├── test_system.sh                  # 시스템 테스트
│   ├── install_python_deps.sh          # Python 의존성 설치
│   └── debug_nodes_jetson.sh           # Jetson 노드 디버깅
├── 📁 docs/                            # 문서 및 가이드
│   ├── JETSON_GUIDE.md                 # Jetson 플랫폼 상세 가이드
│   └── GETTING_STARTED_JETSON.md       # Jetson 빠른 시작
└── � PROJECT_SUMMARY.md               # 프로젝트 개요 및 요약
```
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

## 🚀 설치 및 실행 가이드

### 📋 사전 요구사항

| 구성요소 | 최소 요구사항 | 권장 사양 |
|---------|--------------|----------|
| **OS** | Ubuntu 20.04 | Ubuntu 22.04 LTS |
| **ROS2** | Foxy | Humble Hawksbill |
| **Python** | 3.8+ | 3.10+ |
| **GPU** | 선택사항 | NVIDIA GTX 1060+ |
| **RAM** | 4GB | 8GB+ |
| **Storage** | 20GB | 50GB+ |

### 1️⃣ 시스템 설치

```bash
# 1. 리포지토리 클론
git clone https://github.com/limchanggeon/ros2-delivery-robot.git
cd ros2-delivery-robot

# 2. ROS2 환경 설정 (Ubuntu)
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# 3. 의존성 자동 설치
chmod +x scripts/install_python_deps.sh
./scripts/install_python_deps.sh

# 4. ROS2 패키지 빌드
colcon build --symlink-install
source install/setup.bash
echo "source $(pwd)/install/setup.bash" >> ~/.bashrc
```

### 2️⃣ 통합 관제 시스템 실행

#### 🎯 원클릭 실행 (권장)

```bash
# 전체 시스템 자동 시작
cd "Integrated Control System"
./start_system.sh

# 백그라운드 실행
nohup ./start_system.sh &

# 시스템 상태 확인
./check_status.sh
```

#### 🔧 수동 실행 (디버깅용)

```bash
# 1. 백엔드 서버 시작
cd "Integrated Control System"
python3 -m uvicorn FastAPI:app --host 0.0.0.0 --port 8000 --reload

# 2. 프론트엔드 개발 서버 (별도 터미널)
cd frontend
python3 -m http.server 3000

# 3. 데이터베이스 초기화 (필요시)
python3 -c "
import sqlite3
conn = sqlite3.connect('robot_control.db')
# 테이블 생성 SQL 실행
conn.close()
"
```

### 3️⃣ ROS2 로봇 시스템 실행

#### 🔄 통합 런치 (전체 시스템)

```bash
# 모든 로봇 시스템 + 관제 시스템 동시 실행
ros2 launch Integrated\ Control\ System/launch/integrated_system.launch.py

# 특정 로봇 네임스페이스로 실행
ros2 launch Integrated\ Control\ System/launch/integrated_system.launch.py \
    robot_namespace:=robot_01 \
    backend_host:=localhost \
    backend_port:=8000
```

#### 🧩 개별 시스템 실행

```bash
# 1. 로봇 기본 시스템
ros2 launch delivery_robot_description robot_description.launch.py
ros2 launch delivery_robot_control control.launch.py

# 2. 네비게이션 시스템 
ros2 launch delivery_robot_navigation navigation.launch.py

# 3. 인식 시스템 (YOLOv8)
ros2 launch delivery_robot_perception perception.launch.py

# 4. 미션 관리 시스템
ros2 launch delivery_robot_mission mission.launch.py

# 5. 보안 인증 시스템
ros2 launch delivery_robot_security security.launch.py

# 6. 관제 시스템 노드들
ros2 run delivery_robot_perception status_publisher_node \
    --ros-args -r __ns:=/robot_01
ros2 run delivery_robot_perception web_bridge_node \
    --ros-args -r __ns:=/robot_01 -p backend_host:=localhost
```

### 4️⃣ 시스템 접속 및 사용법

#### 🌐 웹 인터페이스

| 서비스 | URL | 설명 |
|--------|-----|------|
| **메인 대시보드** | http://localhost:8000 | 통합 관제 대시보드 |
| **API 문서** | http://localhost:8000/docs | Swagger UI API 문서 |
| **WebSocket 테스트** | http://localhost:8000/websocket-test | 실시간 통신 테스트 |
| **시스템 상태** | http://localhost:8000/health | 시스템 헬스 체크 |

#### 🔌 API 엔드포인트

```bash
# WebSocket 연결
ws://localhost:8000/ws/ui

# REST API 예시
curl http://localhost:8000/api/robots          # 로봇 목록
curl http://localhost:8000/api/telemetry       # 텔레메트리 데이터  
curl http://localhost:8000/api/missions        # 미션 목록
curl -X POST http://localhost:8000/api/command # 로봇 명령 전송
```

#### 🎮 대시보드 기능 상세

1. **로봇 선택 패널**
   - 드롭다운에서 모니터링할 로봇 선택
   - 연결 상태 실시간 표시

2. **텔레메트리 모니터링**
   - 배터리 레벨 (게이지 + 그래프)
   - CPU/메모리 사용률
   - 네트워크 신호 강도
   - 로봇 속도 및 위치

3. **2D 맵 시각화**
   - Leaflet.js 기반 인터랙티브 맵
   - 실시간 로봇 위치 마커
   - 경로 히스토리 표시
   - 목적지 설정 기능

4. **비디오 스트리밍**
   - 멀티 카메라 지원 (최대 4개)
   - WebRTC 실시간 스트리밍
   - 풀스크린 모드
   - 스냅샷 캡처

5. **원격 제어 패널**
   - 미션 시작/정지/일시정지
   - 비상 정지 (Emergency Stop)
   - 홈 포지션 복귀
   - 충전 스테이션 이동
   - 커스텀 명령 전송

6. **미션 관리**
   - 드래그&드롭 미션 에디터
   - 미션 템플릿 라이브러리
   - 실행 히스토리 및 통계
   - 스케줄링 기능

- Ubuntu 22.04 LTS
- ROS 2 Humble
- Python 3.8+
- CUDA (GPU 사용 시, 선택사항)

### 자동 설치 및 실행

```bash
# 프로젝트 디렉토리로 이동
cd ros2-delivery-robot

# 자동 빌드 및 실행
chmod +x scripts/build_and_run.sh
./scripts/build_and_run.sh
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
chmod +x scripts/test_system.sh
./scripts/test_system.sh
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

⭐ 이 프로젝트가 도움이 되었다면 스타를 눌러주세요!## 🧪 테스트 및 검증

### 자동화 테스트 스크립트

프로젝트에는 다양한 테스트 스크립트가 포함되어 있습니다:

```bash
# 전체 시스템 테스트
./scripts/test_system.sh

# 개별 컴포넌트 테스트
./scripts/test_system_jetson.sh  # Jetson 플랫폼용
./scripts/debug_nodes_jetson.sh  # 노드별 디버깅

# 관제 시스템 상태 확인
cd "Integrated Control System"
./check_status.sh
```

### 단계별 테스트 가이드

#### 1. 기본 환경 테스트
```bash
# ROS2 환경 확인
ros2 doctor
ros2 topic list
ros2 node list

# Python 패키지 확인
python3 -c "import fastapi, uvicorn, websockets; print('✅ 모든 패키지 정상')"
```

#### 2. 로봇 시스템 테스트
```bash
# 각 노드 개별 실행 테스트
ros2 run delivery_robot_perception yolo_inference_node
ros2 run delivery_robot_security authentication_node
ros2 run delivery_robot_mission mission_control_node

# 토픽 발행 상태 확인
ros2 topic echo /robot_01/battery_status
ros2 topic echo /robot_01/odom
ros2 topic echo /robot_01/detection_result
```

#### 3. 관제 시스템 테스트
```bash
# 백엔드 API 테스트
curl -X GET http://localhost:8000/health
curl -X GET http://localhost:8000/api/robots

# WebSocket 연결 테스트
python3 -c "
import asyncio
import websockets

async def test_ws():
    uri = 'ws://localhost:8000/ws/ui'
    async with websockets.connect(uri) as websocket:
        await websocket.send('ping')
        response = await websocket.recv()
        print(f'WebSocket 응답: {response}')

asyncio.run(test_ws())
"
```

#### 4. 통합 테스트
```bash
# 전체 시스템 연동 테스트
ros2 launch Integrated\ Control\ System/launch/integrated_system.launch.py

# 5분 후 자동 종료되는 테스트
timeout 300 ros2 launch delivery_robot_mission full_system_launch.py
```

## 🔧 설정 및 커스터마이징

### 주요 설정 파일

#### ROS2 시스템 설정
```bash
# 네비게이션 파라미터
src/delivery_robot_navigation/config/nav2_params.yaml

# EKF 센서 융합 설정  
src/delivery_robot_navigation/config/ekf.yaml

# YOLO 모델 설정
src/delivery_robot_perception/config/yolo_config.yaml

# QR 인증 설정
src/delivery_robot_security/config/qr_params.yaml

# 미션 관리 설정
src/delivery_robot_mission/config/mission_params.yaml
```

#### 관제 시스템 설정
```bash
# 백엔드 환경 변수
Integrated Control System/.env

# 데이터베이스 스키마
Integrated Control System/datamodel.json

# 프론트엔드 설정
Integrated Control System/frontend/config.js
```

### 환경 변수 설정

`.env` 파일을 생성하여 시스템을 커스터마이징할 수 있습니다:

```bash
cd "Integrated Control System"
cat > .env << EOF
# 서버 설정
BACKEND_HOST=0.0.0.0
BACKEND_PORT=8000
FRONTEND_PORT=3000

# 데이터베이스 설정
DATABASE_URL=sqlite:///robot_control.db
DB_POOL_SIZE=20
DB_MAX_OVERFLOW=30

# WebSocket 설정
WS_HEARTBEAT_INTERVAL=30
WS_MAX_CONNECTIONS=100

# 로봇 설정
DEFAULT_ROBOT_NAMESPACE=robot_01
TELEMETRY_RATE=2.0
MAX_RETRY_ATTEMPTS=10

# 보안 설정
ENABLE_CORS=true
ALLOWED_ORIGINS=["http://localhost:3000", "http://localhost:8000"]
JWT_SECRET_KEY=your-secret-key-here

# 로깅 설정
LOG_LEVEL=INFO
LOG_FILE=logs/backend.log
MAX_LOG_SIZE=10MB
EOF
```

### 로봇별 커스터마이징

```yaml
# robot_config.yaml 예시
robot_01:
  name: "배송로봇 #1"
  model: "DeliveryBot-Pro"
  max_speed: 1.5  # m/s
  battery_capacity: 5000  # mAh
  camera_count: 2
  sensors:
    - lidar
    - camera
    - gps
    - imu

robot_02:
  name: "배송로봇 #2" 
  model: "DeliveryBot-Lite"
  max_speed: 1.0
  battery_capacity: 3000
  camera_count: 1
  sensors:
    - camera
    - gps
```

## 🚨 문제 해결 가이드

### 일반적인 문제와 해결책

#### 1. ROS2 환경 문제
```bash
# 문제: ros2 명령어를 찾을 수 없음
# 해결: ROS2 환경 재설정
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# 문제: 패키지를 찾을 수 없음
# 해결: 워크스페이스 재빌드
colcon build --symlink-install
source install/setup.bash
```

#### 2. 네트워크 연결 문제
```bash
# 문제: WebSocket 연결 실패
# 해결: 포트 확인 및 방화벽 설정
sudo ufw allow 8000
sudo ufw allow 3000
netstat -tulpn | grep :8000

# 문제: 로봇과 통신 안됨
# 해결: ROS_DOMAIN_ID 확인
export ROS_DOMAIN_ID=0
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
```

#### 3. 패키지 의존성 문제
```bash
# 문제: Python 패키지 누락
# 해결: 수동 설치
pip3 install fastapi uvicorn websockets psutil

# 문제: ROS2 패키지 누락
# 해결: rosdep으로 의존성 설치
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

#### 4. 성능 문제
```bash
# 문제: 높은 CPU 사용률
# 해결: 텔레메트리 주기 조정
ros2 param set /robot_01/status_publisher publish_rate 1.0

# 문제: 메모리 부족
# 해결: 불필요한 노드 종료
ros2 node list
ros2 lifecycle set /unnecessary_node shutdown
```

#### 5. 대시보드 접속 문제
```bash
# 문제: 대시보드가 로드되지 않음
# 해결: 브라우저 캐시 삭제 및 포트 확인
curl -I http://localhost:8000
curl -I http://localhost:3000

# 문제: WebSocket 연결 끊김
# 해결: 연결 설정 확인
./check_status.sh
tail -f logs/backend.log
```

### 로그 분석

#### 시스템 로그 위치
```bash
# ROS2 로그
~/.ros/log/

# 관제 시스템 로그
Integrated Control System/logs/

# 시스템 로그
/var/log/syslog
```

#### 유용한 디버깅 명령어
```bash
# ROS2 노드 상태 확인
ros2 node list
ros2 topic list
ros2 service list

# 네트워크 연결 확인  
ss -tulpn | grep :8000
ping localhost

# 프로세스 상태 확인
ps aux | grep python
ps aux | grep ros2

# 리소스 사용량 확인
htop
df -h
free -h
```

## 📚 추가 문서

- **[docs/JETSON_GUIDE.md](docs/JETSON_GUIDE.md)**: NVIDIA Jetson 플랫폼 설치 가이드
- **[docs/GETTING_STARTED_JETSON.md](docs/GETTING_STARTED_JETSON.md)**: Jetson 빠른 시작 가이드  
- **[PROJECT_SUMMARY.md](PROJECT_SUMMARY.md)**: 프로젝트 전체 요약
- **[models/README.md](models/README.md)**: YOLOv8 모델 상세 정보
- **[Integrated Control System/README.md](Integrated%20Control%20System/README.md)**: 관제 시스템 상세 가이드

## 🤝 기여 가이드

### 개발 환경 설정
```bash
# 개발용 브랜치 생성
git checkout -b feature/new-feature

# 개발 의존성 설치
pip3 install black flake8 pytest

# 코드 포맷팅
black src/
flake8 src/

# 테스트 실행
pytest tests/
```

### 기여 방법
1. Fork 프로젝트
2. Feature 브랜치 생성
3. 변경사항 커밋
4. Pull Request 생성

## 📞 지원 및 문의

- **Issues**: [GitHub Issues](https://github.com/limchanggeon/ros2-delivery-robot/issues)
- **Discussions**: [GitHub Discussions](https://github.com/limchanggeon/ros2-delivery-robot/discussions)
- **Email**: limchanggeon@gmail.com

## 📄 라이선스

이 프로젝트는 MIT 라이선스 하에 배포됩니다. 자세한 내용은 [LICENSE](LICENSE) 파일을 참조하세요.

## 🎯 로드맵

### 완료된 기능 ✅
- [x] ROS2 기반 자율주행 시스템
- [x] YOLOv8 객체 인식 통합
- [x] 웹 기반 실시간 모니터링 대시보드
- [x] 다중 로봇 플릿 관리
- [x] WebSocket 실시간 통신
- [x] SQLite 데이터베이스 통합
- [x] 미션 관리 시스템

### 개발 예정 기능 🚧
- [ ] WebRTC 기반 실시간 비디오 스트리밍
- [ ] 음성 명령 인터페이스
- [ ] AI 기반 경로 최적화
- [ ] 클라우드 백엔드 연동
- [ ] 모바일 앱 개발
- [ ] 다국어 지원
- [ ] Docker 컨테이너화
- [ ] Kubernetes 배포 지원

### 장기 계획 🎯
- [ ] 머신러닝 기반 예측 유지보수
- [ ] 블록체인 기반 배송 추적
- [ ] AR/VR 원격 조작 인터페이스
- [ ] 5G 네트워크 최적화
- [ ] 대규모 플릿 관리 (100+ 로봇)

---

<p align="center">
  <strong>🤖 Made with ❤️ for autonomous delivery robots</strong><br>
  <em>ROS2 + NARCHON Integration Project</em>
</p>