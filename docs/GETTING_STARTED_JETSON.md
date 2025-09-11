# ROS2 Delivery Robot — Jetson Orin Nano 시작 가이드 (초보자용)

이 문서는 Jetson Orin Nano(또는 유사한 Jetson 보드) 위에서 이 `ros2-delivery-robot` 프로젝트를 처음부터 실행할 수 있도록, 하드웨어 연결에서부터 빌드·실행·문제해결까지 아주 자세하게 안내합니다. 초보자도 따라 할 수 있게 단계별로, 필요한 명령어를 그대로 복사해서 쓸 수 있도록 만들었습니다.

주의: 이 가이드는 Ubuntu 20.04 + ROS 2 Foxy (JetPack 5.x의 호환성 확인 필요)를 기준으로 작성되었습니다. JetPack/OS 버전에 따라 PyTorch wheel과 일부 명령이 달라질 수 있으니, PyTorch 설치 섹션을 주의해서 읽으세요.

---

## 체크리스트 (한눈에)

- [ ] Jetson Orin Nano 개발 키트 준비 및 전원 연결
- [ ] 키보드/마우스/모니터 또는 SSH 접속 준비
- [ ] microSD 또는 JetPack으로 OS 설치 및 초기 설정 완료
- [ ] ROS 2 Foxy 설치 및 환경 설정
- [ ] Git으로 프로젝트 클론
- [ ] 모델 파일(`models/yolov8_best.pt`)이 `models/`에 있는지 확인
- [ ] Python 의존성 설치 (CUDA PyTorch 포함)
- [ ] colcon 빌드 (젯슨 전용 빌드 스크립트 사용)
- [ ] 시스템 테스트 (개별 노드 → 전체 런치)

---

## 1부 — 하드웨어 준비 (언제 무엇을 연결하나)

목적: Jetson 보드를 안전하게 부팅하고, 카메라와 기타 장치를 연결하여 ROS 노드를 실행할 준비를 합니다.

1. 패키지 확인
   - Jetson Orin Nano Developer Kit(또는 유사 보드)
   - 공식 전원 어댑터(개발 보드 권장 사양 사용)
   - microSD (필요한 경우), 또는 JetPack이 설치된 eMMC
   - USB 키보드/마우스, HDMI 모니터(또는 headless 환경이면 SSH)
   - USB 웹캠 또는 CSI 카메라 (카메라가 필요할 경우)

2. 전원 및 주변기기 연결 순서
   - 전원 어댑터를 연결하기 전에 microSD(또는 eMMC 이미지)가 준비됐는지 확인합니다.
   - HDMI와 USB 키보드/마우스를 연결합니다(로컬에서 설정할 경우).
   - 카메라 연결: USB 웹캠은 USB 포트, CSI 카메라면 레인(케이블)을 정확히 장착합니다.
   - 전원 연결 → 보드 부팅

3. 네트워크/SSH
   - 로컬에서 모니터가 없으면 SSH로 접속하세요.
   - Jetson의 IP 확인 (예: `ip a` 또는 라우터에서 확인)

4. 권한 설정(카메라 및 비디오 장치)

```bash
# 비디오 그룹에 현재 사용자 추가 (로그아웃/재로그인 필요)
sudo usermod -a -G video $USER
# USB 카메라 디바이스 확인
ls /dev/video*
# v4l2 도구가 필요하면 설치
sudo apt install v4l-utils
v4l2-ctl --list-devices
```

팁: CSI 카메라를 사용할 때는 제조사 가이드와 JetPack 드라이버가 필요할 수 있습니다.

---

## 2부 — OS·ROS 설치 및 초기 환경

1. Ubuntu + JetPack 설치
   - NVIDIA 공식 문서(Developer site) 따라 JetPack 설치.
   - JetPack이 CUDA 및 드라이버를 설치합니다.

2. ROS 2 Foxy 설치 (Ubuntu 20.04 기준)

```bash
sudo apt update
sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list'
sudo apt update
sudo apt install -y ros-foxy-desktop python3-argcomplete

# 환경 설정
source /opt/ros/foxy/setup.bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
```

3. 필수 빌드 도구

```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init || true
rosdep update
```

---

## 3부 — 프로젝트 클론 및 파일 확인

1. 프로젝트 클론

```bash
cd ~/ros2_ws
git clone https://github.com/limchanggeon/ros2-delivery-robot.git
cd ros2-delivery-robot
```

2. 저장소 구조 간단 확인

```bash
ls -la
ls -la src
```

3. 모델 파일 위치 확인

```bash
# 프로젝트 루트에 models/yolov8_best.pt 가 있어야 합니다.
ls -la models/
```

- 만약 `models/yolov8_best.pt`가 없다면, 로컬에서 모델을 복사하거나 `models` 디렉토리에 업로드하세요.

---

## 4부 — Python 의존성 및 PyTorch (젯슨용 CUDA) 설치

> 매우 중요: 젯슨에서는 GPU 가속 PyTorch(ARM/aarch64용 wheel)를 설치해야 합니다. CPU wheel을 강제로 설치하면 CUDA가 동작하지 않을 수 있습니다.

1. 프로젝트 제공 스크립트 사용

```bash
chmod +x install_python_deps.sh
./install_python_deps.sh
```

스크립트 내용 요약:
- 젯슨을 감지하면 NVIDIA 공식 PyTorch wheel(`torch-2.0.0+nv23.05-cp38...`)을 다운로드하여 설치하도록 설계되어 있습니다.
- 문제가 생기면 수동으로 wheel을 다운로드하여 설치하세요.

2. 수동 설치 예 (필요한 경우)

```bash
# 예: JetPack 5.x (사용자 환경에 맞는 wheel 확인 필요)
wget https://developer.download.nvidia.com/compute/redist/jp/v50/pytorch/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl
pip3 install --user torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl
pip3 install --user torchvision==0.15.1
```

3. CUDA 작동 확인

```bash
python3 -c "import torch; print('PyTorch', torch.__version__, 'CUDA:', torch.cuda.is_available())"
```

- 출력에서 `CUDA: True` 이어야 GPU를 사용합니다.

4. 추가 Python 패키지

`install_python_deps.sh`는 다음 패키지를 설치합니다: numpy, opencv-python, ultralytics, requests, geopy, pyserial, cryptography, qrcode, pyzbar, pillow, matplotlib 등.

---

## 5부 — 빌드 (colcon) 및 설치

1. 젯슨 전용 빌드 (프로젝트에 포함된 스크립트 사용 권장)

```bash
chmod +x build_and_run_jetson.sh
./build_and_run_jetson.sh
```

- 이 스크립트는 메모리 제약을 고려해 `--parallel-workers 2` 및 `--executor sequential`을 사용합니다.
- 빌드가 실패하면 로그를 확인하고, 필요시 빌드 폴더를 지우고 재시도하세요:

```bash
rm -rf build install log
./build_and_run_jetson.sh
```

2. 빌드 후 환경 설정

```bash
source install/setup.bash
```

3. 빌드가 완료되었는지 확인

```bash
# 설치된 launch 파일 확인
ros2 pkg prefix delivery_robot_mission
find $(ros2 pkg prefix delivery_robot_mission) -name "*.launch.py"

# 패키지 내 실행 파일(콘솔 스크립트) 확인
ros2 pkg executables delivery_robot_perception
ros2 pkg executables delivery_robot_mission
```

---

## 6부 — 실행 순서 (권장)

1. 시스템 모니터로 시작 (간단한 노드 확인)

```bash
ros2 run delivery_robot_mission system_monitor_node
```

2. 카메라 노드 실행 (카메라 연결 확인용)

```bash
ros2 run delivery_robot_perception camera_driver_node
```

3. YOLO 추론 노드 (카메라 & 모델 필요)

```bash
ros2 run delivery_robot_perception yolo_inference_node
```

4. 전체 시스템 런치 (RViz 포함)

```bash
ros2 launch delivery_robot_mission full_system_launch.py

# 헤드리스 또는 RViz 없는 환경
ros2 launch delivery_robot_mission full_system_launch.py use_rviz:=false
```

---

## 7부 — 문제해결 가이드 (초보자용 FAQ)

Q: `file 'full_system_launch.py' was not found in the share directory` 오류가 뜹니다.

A: 빌드 후 `install/<pkg>/share/<pkg>/launch/`에 런치 파일이 복사되지 않은 상태입니다. 빠른 해결:

```bash
# 임시 수동 복사 (프로젝트 루트에서)
chmod +x quick_fix_launch.sh
./quick_fix_launch.sh
source install/setup.bash
ros2 launch delivery_robot_mission full_system_launch.py
```

또는 전체 재빌드:

```bash
rm -rf build install log
./build_and_run_jetson.sh
```

Q: `ros2 run <pkg> <node>` 실행 시 "executable not found"가 뜹니다.

A: `setup.py`의 `entry_points`가 설치되지 않았거나 빌드가 정상적이지 않은 경우입니다. 해결:

```bash
# 설치된 콘솔 스크립트 확인
ros2 pkg executables delivery_robot_perception
# 패키지 재빌드
colcon build --packages-select delivery_robot_perception --symlink-install
source install/setup.bash
```

Q: OpenCV 관련 모듈 오류

A: `opencv-python`이 설치되지 않았습니다. 젯슨에서는 `opencv-python` 대신 `opencv-python-headless`를 권장할 때가 있습니다:

```bash
pip3 install --user opencv-python
# 또는
pip3 install --user opencv-python-headless
```

Q: PyTorch가 `CUDA: False`로 나옵니다.

A: PyTorch가 CPU 전용으로 설치된 경우입니다. `install_python_deps.sh`가 Jetson을 인식하여 CUDA wheel을 설치하도록 되어 있지만, 실패할 수 있습니다. 수동으로 NVIDIA wheel을 설치하세요 (아래 예시는 JetPack 5.x용 예시입니다 — 사용 환경에 맞는 파일을 사용하세요):

```bash
wget https://developer.download.nvidia.com/compute/redist/jp/v50/pytorch/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl
pip3 install --user torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl
pip3 install --user torchvision==0.15.1
python3 -c "import torch; print('CUDA available:', torch.cuda.is_available())"
```

Q: 메모리 부족 또는 빌드 실패

A: 젯슨은 메모리가 제한적입니다. 권장 조치:

- 스왑 파일 추가 (임시)

```bash
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

- 빌드 병렬도 낮추기

```bash
export MAKEFLAGS="-j1"
colcon build --symlink-install --parallel-workers 1 --executor sequential
```

---

## 8부 — 유용한 디버깅 명령어 모음

```bash
# ROS2 노드/토픽 확인
ros2 node list
ros2 topic list
ros2 topic echo /yolo/detections

# 특정 패키지의 콘솔 실행 파일 확인
ros2 pkg executables delivery_robot_perception

# 설치된 launch 파일 찾기
ros2 pkg prefix delivery_robot_mission
find $(ros2 pkg prefix delivery_robot_mission) -name "*.launch.py"

# 파이썬 패키지 확인
python3 -c "import torch, cv2, ultralytics; print(torch.__version__, torch.cuda.is_available(), cv2.__version__)"
```

---

## 9부 — 추가 팁

- 개발 초반에는 `use_rviz:=false`로 실행하여 GUI 의존성 문제를 피하세요.
- 모델 파일 변경시 `install`에 자동 반영되지 않으므로 `src` → `install`로 수동 복사하거나 패키지 재설치 필요.
- PyTorch/torchvision 버전은 JetPack/OS에 민감합니다. NVIDIA 문서와 JetPack 릴리즈 정보를 우선 확인하세요.

---

필요하면 이 문서를 더 줄이거나, 패키지별(Perception, Navigation, Security 등) 상세 가이드로 분할해 드리겠습니다. 어떤 형태로 더 정리할지 알려주세요!