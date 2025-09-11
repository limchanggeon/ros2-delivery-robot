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
# 🚀 JETSON 플랫폼 설치 및 설정 가이드

> NVIDIA Jetson 시리즈에서 ROS2 배송 로봇 + NARCHON 관제 시스템을 실행하기 위한 완전한 가이드

## 📋 지원 플랫폼

| 모델 | 아키텍처 | RAM | 권장 여부 |
|------|---------|-----|----------|
| **Jetson Nano** | ARM64 | 4GB | ⚠️ 최소 사양 |
| **Jetson Xavier NX** | ARM64 | 8GB | ✅ 권장 |
| **Jetson AGX Xavier** | ARM64 | 32GB | ✅ 최고 성능 |
| **Jetson Orin Nano** | ARM64 | 8GB | 🔥 최신 권장 |
| **Jetson Orin** | ARM64 | 32GB+ | 🔥 최신 |

## 🛠️ 1단계: 기본 시스템 설정

### JetPack SDK 설치

```bash
# JetPack 5.0+ 권장 (Ubuntu 20.04 기반)
# NVIDIA SDK Manager를 통해 설치하거나
# 미리 구성된 이미지 다운로드

# 시스템 업데이트
sudo apt update && sudo apt upgrade -y

# 기본 개발 도구 설치
sudo apt install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    python3-pip \
    python3-dev \
    pkg-config \
    v4l-utils \
    software-properties-common
```

### CUDA 및 cuDNN 확인

```bash
# CUDA 버전 확인
nvcc --version
nvidia-smi

# cuDNN 확인 (JetPack에 포함)
ls /usr/include/cudnn*
cat /usr/include/cudnn_version.h | grep CUDNN_MAJOR -A 2

# 환경 변수 설정
echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc
echo 'export CUDA_ROOT=/usr/local/cuda' >> ~/.bashrc
source ~/.bashrc
```

## 🤖 2단계: ROS2 설치 (ARM64)

### ROS2 Humble 설치 (권장)

```bash
# 로케일 설정
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ROS2 APT 리포지토리 추가
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl gnupg lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS2 Humble 설치
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop

# 개발 도구 설치
sudo apt install -y \
    python3-flake8-docstrings \
    python3-pip \
    python3-pytest-cov \
    ros-dev-tools \
    python3-rosdep \
    python3-colcon-common-extensions

# 환경 설정
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# rosdep 초기화
sudo rosdep init
rosdep update
```

### ROS2 Foxy 설치 (호환성을 위해)

```bash
# Foxy 설치 (Ubuntu 20.04에서)
sudo apt install ros-foxy-desktop python3-argcomplete

# Humble과 Foxy 동시 사용 설정
echo "# ROS2 환경 선택" >> ~/.bashrc
echo "alias ros2-humble='source /opt/ros/humble/setup.bash'" >> ~/.bashrc
echo "alias ros2-foxy='source /opt/ros/foxy/setup.bash'" >> ~/.bashrc
source ~/.bashrc
```

## 🐍 3단계: Python 의존성 설치

### PyTorch for Jetson 설치

```bash
# Jetson에 최적화된 PyTorch 설치
# JetPack 5.x (Python 3.8)의 경우
wget https://developer.download.nvidia.com/compute/redist/jp/v50/pytorch/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl
pip3 install torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl

# torchvision 설치
pip3 install torchvision==0.15.1

# CUDA 지원 확인
python3 -c "import torch; print(f'PyTorch {torch.__version__} - CUDA: {torch.cuda.is_available()}')"
```

### 추가 Python 패키지

```bash
# 컴퓨터 비전 및 AI 패키지
pip3 install -U \
    ultralytics \
    opencv-python \
    opencv-contrib-python \
    pyzbar[scripts] \
    Pillow \
    numpy \
    scipy \
    matplotlib

# 웹 서비스 패키지
pip3 install -U \
    fastapi \
    uvicorn \
    pydantic \
    jinja2 \
    python-multipart \
    websockets \
    requests
```

## 🏗️ 4단계: 프로젝트 설정

### 저장소 클론 및 빌드

```bash
# 작업 공간 생성
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

# 프로젝트 클론
git clone https://github.com/limchanggeon/ros2-delivery-robot.git
cd ros2-delivery-robot

# Jetson 테스트 스크립트 실행
chmod +x test_system_jetson.sh
./test_system_jetson.sh

# Python 의존성 설치
chmod +x install_python_deps.sh
./install_python_deps.sh

# 의존성 해결
cd ~/robot_ws
rosdep install --from-paths src --ignore-src -r -y --skip-keys="libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python3-opencv"

# 빌드 (메모리 절약을 위해 단일 스레드)
colcon build --symlink-install --parallel-workers 1

# 환경 설정
echo "source ~/robot_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## ⚡ 5단계: Jetson 최적화 설정

### 성능 모드 설정

```bash
# 최대 성능 모드
sudo nvpmodel -m 0
sudo jetson_clocks

# 성능 상태 확인
sudo nvpmodel -q
jetson_clocks --show

# 팬 속도 최대 설정 (과열 방지)
echo 255 | sudo tee /sys/devices/pwm-fan/target_pwm
```

### 메모리 최적화

```bash
# 스왑 파일 생성 (8GB 권장)
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

# GPU 메모리 분할 설정 (Jetson Nano의 경우)
# /boot/firmware/config.txt 편집 (필요시)
# gpu_mem=128

# 메모리 사용량 확인
free -h
nvidia-smi
```

### 전력 관리 설정

```bash
# 전력 모니터링 도구 설치
sudo apt install jetson-stats
sudo systemctl restart jtop.service

# jtop 실행하여 모니터링
jtop
```

## 🚀 6단계: 시스템 실행

### 빠른 시작

```bash
# 환경 설정
source /opt/ros/humble/setup.bash
source ~/robot_ws/install/setup.bash

# 전체 시스템 실행 (Jetson 최적화)
chmod +x build_and_run_jetson.sh
./build_and_run_jetson.sh
```

### 개별 컴포넌트 실행

```bash
# 1. 카메라 노드 (첫 번째 터미널)
ros2 run delivery_robot_perception camera_driver_node

# 2. YOLO 추론 노드 (두 번째 터미널)
ros2 run delivery_robot_perception yolo_inference_node

# 3. 미션 제어 노드 (세 번째 터미널)
ros2 run delivery_robot_mission mission_control_node

# 4. 인증 노드 (네 번째 터미널)
ros2 run delivery_robot_security authentication_node

# 5. 웹 브릿지 (다섯 번째 터미널)
cd ~/robot_ws/src/ros2-delivery-robot/Integrated\ Control\ System/
python3 web_bridge_node.py
```

### 통합 실행

```bash
# 전체 시스템 런치
ros2 launch delivery_robot_mission full_system_launch.py

# 헤드리스 모드 (RViz 없이)
ros2 launch delivery_robot_mission full_system_launch.py use_rviz:=false

# 디버그 모드
ros2 launch delivery_robot_mission full_system_launch.py log_level:=debug
```

## 🔧 7단계: 문제 해결

### 일반적인 문제

#### 1. CUDA 메모리 오류

```bash
# GPU 메모리 사용량 확인
nvidia-smi

# CUDA 메모리 해제
python3 -c "
import torch
torch.cuda.empty_cache()
print('CUDA cache cleared')
"

# 환경 변수 설정
export CUDA_LAUNCH_BLOCKING=1
export TORCH_CUDA_ARCH_LIST="5.3;6.2;7.2"  # Jetson 아키텍처
```

#### 2. PyTorch 모델 로딩 실패

```bash
# 모델 경로 확인
ls -la ~/robot_ws/src/ros2-delivery-robot/models/

# 권한 설정
chmod +r ~/robot_ws/src/ros2-delivery-robot/models/yolov8_best.pt

# TensorRT 최적화 (선택사항)
python3 -c "
from ultralytics import YOLO
model = YOLO('models/yolov8_best.pt')
model.export(format='engine', device=0)  # TensorRT 변환
"
```

#### 3. 카메라 인식 문제

```bash
# USB 카메라 확인
ls /dev/video*
v4l2-ctl --list-devices

# 카메라 테스트
v4l2-ctl --device=/dev/video0 --info
ffmpeg -f v4l2 -list_formats all -i /dev/video0

# 권한 설정
sudo usermod -a -G video $USER
sudo chmod 666 /dev/video*
```

#### 4. ROS2 통신 문제

```bash
# DDS 설정 최적화
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
export CYCLONEDX_URI=udp://localhost:7400

# 멀티캐스트 확인
sudo apt install net-tools
netstat -g

# 방화벽 설정
sudo ufw disable  # 개발 중에만
```

#### 5. 메모리 부족 오류

```bash
# 스왑 사용량 확인
swapon --show
free -h

# 프로세스 메모리 사용량 확인
top -o %MEM
ps aux --sort=-%mem | head

# 메모리 정리
sudo sh -c 'echo 1 > /proc/sys/vm/drop_caches'
```

## 📊 8단계: 모니터링 및 디버깅

### 시스템 모니터링

```bash
# Jetson 전용 모니터링
jtop  # GPU, CPU, 메모리, 온도 실시간 확인

# 온도 모니터링
watch -n 1 "cat /sys/devices/virtual/thermal/thermal_zone*/temp"

# 전력 소비 확인
cat /sys/bus/i2c/drivers/ina3221x/*/iio:device*/in_power*_input
```

### ROS2 디버깅

```bash
# 노드 상태 확인
ros2 node list
ros2 topic list
ros2 service list

# 토픽 데이터 모니터링
ros2 topic echo /camera/image_raw --field header
ros2 topic echo /yolo/detections
ros2 topic hz /camera/image_raw

# 로그 확인
ros2 run rqt_console rqt_console

# 그래프 시각화
rqt_graph
```

### 성능 벤치마킹

```bash
# YOLO 추론 속도 테스트
python3 -c "
import time
from ultralytics import YOLO
import torch

model = YOLO('models/yolov8_best.pt')
dummy_input = torch.randn(1, 3, 640, 640).cuda()

# 워밍업
for _ in range(10):
    model(dummy_input)

# 벤치마크
start_time = time.time()
for _ in range(100):
    model(dummy_input)
end_time = time.time()

print(f'평균 추론 시간: {(end_time - start_time) / 100 * 1000:.2f}ms')
print(f'FPS: {100 / (end_time - start_time):.1f}')
"
```

## 🔄 9단계: 자동 시작 설정

### systemd 서비스 생성

```bash
# 서비스 파일 생성
sudo tee /etc/systemd/system/delivery-robot.service > /dev/null <<EOF
[Unit]
Description=ROS2 Delivery Robot System
After=network.target graphical-session.target
Wants=graphical-session.target

[Service]
Type=simple
User=$USER
Group=$USER
WorkingDirectory=$HOME/robot_ws
Environment="HOME=$HOME"
Environment="ROS_DOMAIN_ID=0"
Environment="DISPLAY=:0"
ExecStartPre=/bin/bash -c 'source /opt/ros/humble/setup.bash'
ExecStart=/bin/bash -c 'source install/setup.bash && ros2 launch delivery_robot_mission full_system_launch.py use_rviz:=false'
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

# 서비스 활성화
sudo systemctl daemon-reload
sudo systemctl enable delivery-robot.service

# 수동 시작/중지
sudo systemctl start delivery-robot.service
sudo systemctl status delivery-robot.service
sudo systemctl stop delivery-robot.service
```

### 부팅 시 성능 최적화 자동 실행

```bash
# 성능 최적화 스크립트 생성
sudo tee /etc/systemd/system/jetson-performance.service > /dev/null <<EOF
[Unit]
Description=Jetson Performance Optimization
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/bin/bash -c 'nvpmodel -m 0 && jetson_clocks'
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl enable jetson-performance.service
```

## 🌐 10단계: 원격 접속 및 모니터링

### SSH 설정

```bash
# SSH 서버 설치 및 활성화
sudo apt install openssh-server
sudo systemctl enable ssh
sudo systemctl start ssh

# 방화벽 설정
sudo ufw allow ssh
sudo ufw allow 8080  # 웹 인터페이스
sudo ufw --force enable

# SSH 키 기반 인증 설정 (선택사항)
ssh-keygen -t rsa -b 4096
```

### 웹 기반 모니터링

```bash
# 웹 브라우저에서 접속
# http://JETSON_IP:8080

# 네트워크 설정 확인
ip addr show
hostname -I
```

### VNC 설정 (GUI 원격 접속)

```bash
# VNC 서버 설치
sudo apt install tigervnc-standalone-server tigervnc-xorg-extension

# VNC 비밀번호 설정
vncpasswd

# VNC 서버 시작
vncserver :1 -geometry 1920x1080 -depth 24

# 클라이언트에서 접속: JETSON_IP:5901
```

## 📈 11단계: 성능 최적화 고급 팁

### TensorRT 최적화

```bash
# YOLOv8 모델 TensorRT 변환
python3 -c "
from ultralytics import YOLO

# 모델 로드
model = YOLO('models/yolov8_best.pt')

# TensorRT 엔진 생성
model.export(
    format='engine',
    device=0,
    half=True,  # FP16 사용
    workspace=4,  # 4GB 워크스페이스
    verbose=True
)

print('TensorRT 엔진 생성 완료')
"

# 변환된 엔진 사용
ls models/*.engine
```

### CUDA 스트림 최적화

```bash
# CUDA 스트림 설정
export CUDA_DEVICE_ORDER=PCI_BUS_ID
export CUDA_VISIBLE_DEVICES=0

# PyTorch 최적화 설정
python3 -c "
import torch
torch.backends.cudnn.benchmark = True
torch.backends.cudnn.deterministic = False
print('CUDA 최적화 설정 완료')
"
```

### 메모리 사용량 최적화

```bash
# zram 압축 스왑 설정
sudo apt install zram-tools
sudo sed -i 's/#PERCENT=25/PERCENT=50/' /etc/default/zramswap
sudo systemctl restart zramswap

# 커널 매개변수 최적화
echo 'vm.swappiness=10' | sudo tee -a /etc/sysctl.conf
echo 'vm.vfs_cache_pressure=50' | sudo tee -a /etc/sysctl.conf
sudo sysctl -p
```

## 🚨 12단계: 응급 복구 및 백업

### 시스템 백업

```bash
# 중요 설정 백업
mkdir -p ~/backup
cp ~/.bashrc ~/backup/
cp -r ~/robot_ws/src ~/backup/
sudo cp /etc/systemd/system/delivery-robot.service ~/backup/

# SD 카드 이미지 백업 (다른 컴퓨터에서)
# sudo dd if=/dev/sdX of=jetson-backup.img bs=4M status=progress
```

### 응급 복구 모드

```bash
# 서비스 일시 중지
sudo systemctl stop delivery-robot.service

# 로그 확인
journalctl -u delivery-robot.service -f
dmesg | tail -50

# 최소 모드로 시작
source /opt/ros/humble/setup.bash
ros2 run delivery_robot_perception camera_driver_node  # 개별 테스트
```

### 시스템 초기화

```bash
# 작업 공간 초기화
cd ~/robot_ws
rm -rf build install log
colcon build --symlink-install --parallel-workers 1

# Python 캐시 정리
find . -type d -name __pycache__ -exec rm -rf {} +
pip3 cache purge

# CUDA 캐시 정리
rm -rf ~/.nv/ComputeCache/
```

## 📚 추가 리소스 및 참고 자료

### 공식 문서

- [NVIDIA Jetson 개발자 가이드](https://docs.nvidia.com/jetson/)
- [JetPack SDK 문서](https://docs.nvidia.com/jetpack/)
- [ROS2 공식 문서](https://docs.ros.org/en/humble/)
- [PyTorch for Jetson](https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048)

### 커뮤니티 및 지원

- [NVIDIA Developer Forums](https://forums.developer.nvidia.com/c/agx-autonomous-machines/jetson-embedded-systems/)
- [ROS Discourse](https://discourse.ros.org/)
- [Jetson Community Projects](https://developer.nvidia.com/embedded/community/jetson-projects)

### 성능 벤치마크

- [MLPerf Inference 결과](https://mlcommons.org/en/inference-edge-21/)
- [Jetson 벤치마크 도구](https://github.com/NVIDIA-AI-IOT/jetson_benchmarks)

---

## 🎯 성공적인 배포를 위한 체크리스트

- [ ] JetPack 5.0+ 설치 완료
- [ ] CUDA/cuDNN 정상 작동 확인
- [ ] ROS2 Humble 설치 및 환경 설정
- [ ] PyTorch CUDA 지원 확인
- [ ] 모든 Python 의존성 설치
- [ ] 프로젝트 빌드 성공
- [ ] 카메라 인식 정상 동작
- [ ] YOLO 추론 속도 30FPS 이상
- [ ] 웹 인터페이스 접속 가능
- [ ] 자동 시작 서비스 설정
- [ ] 원격 모니터링 환경 구축

**문제 발생 시**: `test_system_jetson.sh` 스크립트를 실행하여 시스템 상태를 진단하세요.

**긴급 지원**: GitHub Issues에 로그와 함께 문제를 보고해주세요.

🚀 **성공적인 Jetson 배포를 위해!**

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

#### 2. PyTorch CUDA 설치 문제
```bash
# 젯슨 오린 나노용 PyTorch CUDA 설치 (JetPack 5.x)
wget https://developer.download.nvidia.com/compute/redist/jp/v50/pytorch/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl
pip3 install torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl
pip3 install torchvision==0.15.1

# CUDA 지원 확인
python3 -c "import torch; print(f'PyTorch {torch.__version__} - CUDA: {torch.cuda.is_available()}')"
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