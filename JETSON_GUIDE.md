# 젯슨 오린 나노 빠른 시작 가이드

이 가이드는 젯슨 오린 나노 Ubuntu 20.04 환경에서 ROS 2 배송 로봇 시스템을 설정하고 실행하는 방법을 설명합니다.

## 🛠️ 사전 요구사항

### 하드웨어
- NVIDIA Jetson Orin Nano Developer Kit
- microSD 카드 (64GB 이상 권장)
- USB 카메라
- 인터넷 연결

### 소프트웨어
- Ubuntu 20.04 LTS (JetPack 5.x)
- ROS 2 Foxy

## 🚀 빠른 설치

### 1. 저장소 클론
```bash
git clone https://github.com/limchanggeon/ros2-delivery-robot.git
cd ros2-delivery-robot
```

### 2. 시스템 테스트
```bash
chmod +x test_system_jetson.sh
./test_system_jetson.sh
```

### 3. ROS 2 Foxy 설치 (필요한 경우)
```bash
# ROS 2 Foxy 설치
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-foxy-desktop python3-argcomplete
```

### 4. 환경 설정
```bash
# ROS 2 환경 설정
source /opt/ros/foxy/setup.bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

# 젯슨 성능 최적화
sudo jetson_clocks
sudo nvpmodel -m 0  # 최대 성능 모드
```

### 5. 의존성 설치 및 빌드
```bash
# Python 의존성 설치
./install_python_deps.sh

# 젯슨 최적화 빌드
./build_and_run_jetson.sh
```

## 🎮 시스템 실행

### 환경 설정
```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
```

### 개별 노드 테스트
```bash
# YOLOv8 객체 인식
ros2 run delivery_robot_perception yolo_inference_node

# 카메라 드라이버 (새 터미널)
ros2 run delivery_robot_perception camera_driver_node

# 임무 제어 (새 터미널)
ros2 run delivery_robot_mission mission_control_node

# QR 코드 인증 (새 터미널)
ros2 run delivery_robot_security authentication_node
```

### 전체 시스템 실행
```bash
# 모든 서브시스템 통합 실행
ros2 launch delivery_robot_mission full_system_launch.py
```

### 헤드리스 모드 (GUI 없이 실행)
```bash
# RViz 없이 실행
ros2 launch delivery_robot_mission full_system_launch.py use_rviz:=false
```

## 🔧 문제 해결

### 일반적인 문제

#### 1. 메모리 부족 오류
```bash
# 스왑 메모리 확인/설정
sudo swapon --show
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

#### 2. PyTorch 설치 문제
```bash
# 젯슨용 PyTorch 설치
wget https://nvidia.box.com/shared/static/fjtbno0vpo676a25cgvuqc1wty0fkkg6.whl -O torch-1.10.0-cp38-cp38-linux_aarch64.whl
pip3 install torch-1.10.0-cp38-cp38-linux_aarch64.whl
```

#### 3. 카메라 인식 문제
```bash
# USB 카메라 확인
ls /dev/video*
v4l2-ctl --list-devices

# 권한 설정
sudo usermod -a -G video $USER
```

#### 4. 빌드 실패
```bash
# 캐시 정리 후 재빌드
rm -rf build install log
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --parallel-workers 1
```

### 성능 최적화

#### GPU 활용
```bash
# CUDA 상태 확인
nvidia-smi

# GPU 메모리 최적화
export CUDA_VISIBLE_DEVICES=0
export TF_FORCE_GPU_ALLOW_GROWTH=true
```

#### 전력 관리
```bash
# 전력 모드 확인
sudo nvpmodel -q

# 최대 성능 모드 설정
sudo nvpmodel -m 0
sudo jetson_clocks
```

## 📊 모니터링

### 시스템 리소스 모니터링
```bash
# 젯슨 전용 모니터링 도구
sudo apt install jetson-stats
jtop
```

### ROS 2 토픽 모니터링
```bash
# 활성 토픽 확인
ros2 topic list

# 토픽 데이터 확인
ros2 topic echo /yolo/detections
ros2 topic echo /camera/image_raw
```

### 로그 확인
```bash
# ROS 2 로그
ros2 run rqt_console rqt_console

# 시스템 로그
journalctl -f
```

## 📱 원격 접속

### SSH 설정
```bash
# SSH 서버 설치
sudo apt install openssh-server

# 원격 접속
ssh username@jetson-ip-address
```

### VNC 설정 (선택사항)
```bash
# VNC 서버 설치
sudo apt install vino
gsettings set org.gnome.Vino enabled true
```

## 🔄 자동 실행 설정

### 시스템 서비스 생성
```bash
# 서비스 파일 생성
sudo nano /etc/systemd/system/delivery-robot.service
```

서비스 파일 내용:
```ini
[Unit]
Description=Delivery Robot System
After=network.target

[Service]
Type=simple
User=jetson
WorkingDirectory=/home/jetson/ros2-delivery-robot
Environment="ROS_DISTRO=foxy"
ExecStartPre=/bin/bash -c 'source /opt/ros/foxy/setup.bash'
ExecStart=/bin/bash -c 'source install/setup.bash && ros2 launch delivery_robot_mission full_system_launch.py use_rviz:=false'
Restart=always

[Install]
WantedBy=multi-user.target
```

서비스 활성화:
```bash
sudo systemctl enable delivery-robot.service
sudo systemctl start delivery-robot.service
```

## 📚 추가 리소스

- [NVIDIA Jetson 공식 문서](https://developer.nvidia.com/embedded/jetson-orin-nano-developer-kit)
- [ROS 2 Foxy 문서](https://docs.ros.org/en/foxy/)
- [프로젝트 GitHub 저장소](https://github.com/limchanggeon/ros2-delivery-robot)

---

문제가 발생하면 `test_system_jetson.sh` 스크립트를 실행하여 시스템 상태를 확인하세요.