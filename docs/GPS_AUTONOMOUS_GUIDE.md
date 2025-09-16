# 🚀 GPS 기반 자율주행 시스템 구축 가이드

ROS2에서 정적 지도와 GPS를 이용한 지리 참조 기반 자율 주행 시스템을 구축하는 완전한 가이드입니다.

## 📋 목차

1. [시스템 개요](#시스템-개요)
2. [필수 조건](#필수-조건)
3. [빠른 시작](#빠른-시작)
4. [시스템 구성 요소](#시스템-구성-요소)
5. [GPS 지도 보정 방법](#gps-지도-보정-방법)
6. [문제 해결](#문제-해결)
7. [고급 설정](#고급-설정)

## 🎯 시스템 개요

이 시스템은 다음과 같은 계층적 구조로 GPS 기반 자율주행을 구현합니다:

```
지구(Earth) → UTM → MAP → ODOM → BASE_LINK
```

### 핵심 기능
- ✅ **정적 지도와 GPS 좌표 정렬**: 항공사진이나 지도 이미지를 실제 지리 좌표에 정확히 매핑
- ✅ **이중 EKF 아키텍처**: 지역/전역 위치 추정을 통한 강력한 센서 융합
- ✅ **드리프트 없는 전역 측위**: GPS 기반으로 누적 오차 없는 장거리 항법
- ✅ **대화형 보정 시스템**: RViz 클릭 한 번으로 지도-GPS 정렬
- ✅ **Nav2 호환성**: 표준 ROS2 항법 스택과 완벽 통합

## 🔧 필수 조건

### 하드웨어 요구사항
- **GPS 수신기**: RTK-GPS 권장 (정확도 향상)
- **IMU 센서**: 9축 IMU (자이로스코프, 가속도계, 지자기센서)
- **휠 엔코더**: 오도메트리 제공
- **컴퓨터**: ROS2 실행 가능한 리눅스 시스템

### 소프트웨어 요구사항
- **ROS2** (Humble/Foxy)
- **robot_localization** 패키지
- **Nav2** 스택
- **Python 3** + 필요 라이브러리

### 설치 명령어
```bash
# ROS2 패키지 설치
sudo apt update
sudo apt install ros-$ROS_DISTRO-robot-localization \
                 ros-$ROS_DISTRO-nav2-map-server \
                 ros-$ROS_DISTRO-nav2-lifecycle-manager \
                 ros-$ROS_DISTRO-tf2-tools \
                 ros-$ROS_DISTRO-rviz2

# Python 의존성
pip3 install pyproj pyyaml
```

## ⚡ 빠른 시작

### 1단계: 지도 준비

먼저 지도 이미지(.jpg, .png)를 ROS 형식으로 변환합니다:

```bash
# 이미지를 .pgm 형식으로 변환 (GIMP 등 사용)
# 흰색=자유공간, 검은색=장애물

# map.yaml 파일 생성 예시:
cat > my_map.yaml << EOF
image: my_map.pgm
resolution: 0.05    # 픽셀당 미터 (정확히 계산 필요!)
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
EOF
```

### 2단계: 센서 확인

필수 토픽들이 발행되는지 확인:

```bash
# GPS 토픽 확인
ros2 topic echo /gps/fix

# IMU 토픽 확인  
ros2 topic echo /imu/data

# 오도메트리 토픽 확인
ros2 topic echo /odom
```

### 3단계: 시스템 시작

```bash
# 빠른 시작 스크립트 실행
./scripts/quick_start_gps_system.sh /path/to/your/map.yaml
```

### 4단계: GPS 지도 보정

1. **RViz 설정**:
   - Fixed Frame을 `map`으로 설정
   - Map Display 추가하여 지도 확인

2. **보정 실행**:
   - GPS 신호 수신 대기 (터미널에서 확인)
   - RViz의 "Publish Point" 도구 선택
   - 지도 위에서 현재 로봇 위치에 해당하는 지점 클릭

3. **보정 완료**:
   - 터미널에 "✅ UTM -> MAP 정적 변환 발행 완료!" 메시지 확인
   - 지도가 GPS 좌표에 정렬됨

## 🔬 시스템 구성 요소

### 1. 지역 EKF (`ekf_local.yaml`)
- **역할**: 연속적인 센서 데이터 융합
- **입력**: 휠 오도메트리 + IMU 각속도/가속도
- **출력**: `odom` → `base_link` 변환
- **특성**: 높은 주파수, 부드럽지만 드리프트 발생

### 2. 전역 EKF (`ekf_global.yaml`)  
- **역할**: 전역 위치 추정 및 드리프트 보정
- **입력**: 지역 오도메트리 + GPS 변환 오도메트리 + IMU 방위각
- **출력**: `map` → `odom` 변환
- **특성**: 중간 주파수, 드리프트 없음

### 3. NavSat Transform (`navsat_transform.yaml`)
- **역할**: GPS 좌표를 ROS 좌표계로 변환
- **입력**: GPS fix + IMU 방위각 + 지역 오도메트리
- **출력**: GPS 기반 오도메트리 메시지
- **특성**: UTM 투영법 사용

### 4. GPS 지도 보정 노드 (`map_ros.py`)
- **역할**: 지도 이미지를 GPS 좌표에 정렬
- **입력**: RViz 클릭 지점 + 현재 GPS 위치
- **출력**: `utm` → `map` 정적 변환
- **특성**: 일회성 보정, 설정값 자동 저장

## 🗺️ GPS 지도 보정 방법

### 대화형 보정 (권장)

이 방법은 현장에서 직관적으로 지도를 GPS 좌표에 정렬할 수 있습니다.

#### 준비 단계
1. GPS 수신기가 안정적인 신호를 받고 있는지 확인
2. 로봇을 지도상에서 식별 가능한 위치에 배치
3. RViz에서 지도가 올바르게 로딩되었는지 확인

#### 보정 실행
```bash
# 보정 노드만 별도 실행 (시스템이 이미 실행 중인 경우)
ros2 run capston_project map_ros.py
```

#### RViz 설정
```
Fixed Frame: map
Add → By display type → Map
Topic: /map
```

#### 클릭 보정
1. RViz 상단의 "Publish Point" 버튼 클릭
2. 지도상에서 현재 로봇이 위치한 지점을 클릭
3. 터미널에서 보정 완료 메시지 확인:
   ```
   🎯 보정 지점 선택됨:
      📍 지도 좌표: (x=45.123, y=23.456) [m]
      🌍 GPS 좌표: (위도=37.123456°, 경도=126.987654°)
      📐 UTM 좌표: (x=323456.789, y=4567890.123) [m]
   ✅ UTM -> MAP 정적 변환 발행 완료!
   ```

### 정적 보정 (자동화된 환경)

반복적인 배포나 무인 시스템에 적합합니다.

#### 1. 기준점 설정
정확한 GPS 좌표를 알고 있는 지도상의 지점을 선택합니다.

#### 2. navsat_transform.yaml 수정
```yaml
navsat_transform:
  ros__parameters:
    wait_for_datum: true
    # 기준점 정보는 런치 파일에서 제공
```

#### 3. 런치 파일에 datum 추가
```python
navsat_transform_node = Node(
    # ... 기존 설정 ...
    parameters=[
        # ... 기존 파라미터 ...
        {
            'datum': [37.123456, 126.987654, 0.0, 'map', 'base_link']
            # [위도, 경도, 방위각, map_frame_id, base_link_frame_id]
        }
    ]
)
```

## 🔧 문제 해결

### GPS 신호 문제

**증상**: GPS 토픽에 데이터가 없거나 품질이 낮음
```bash
# GPS 상태 확인
ros2 topic echo /gps/fix --once

# 정상적인 출력 예시:
# status: 
#   status: 0  # 0=정상, -1=신호없음
# latitude: 37.123456
# longitude: 126.987654
```

**해결방법**:
- 야외 개방된 장소로 이동
- GPS 안테나 위치 조정
- RTK-GPS 사용 시 Base Station 연결 확인

### TF 변환 문제

**증상**: "Transform timeout" 또는 "Could not transform" 에러

```bash
# TF 트리 상태 확인
ros2 run tf2_tools view_frames

# TF 변환 확인
ros2 run tf2_ros tf2_echo utm map
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

**해결방법**:
- 각 EKF 노드가 정상 실행되는지 확인
- 센서 토픽들이 올바른 frame_id를 갖는지 확인
- 설정 파일의 frame 이름 일치성 점검

### 지도 해상도 문제

**증상**: 로봇이 목표 지점을 지나치거나 못 미치는 현상

**해결방법**:
1. 지도에서 알려진 거리 측정 (예: 건물 폭 20m)
2. 이미지에서 해당 구간의 픽셀 수 측정 (예: 400px)
3. 해상도 계산: 20m ÷ 400px = 0.05 m/pixel
4. map.yaml의 resolution 값 수정

### IMU 방위각 문제

**증상**: 로봇이 잘못된 방향으로 회전하거나 GPS 융합이 불안정

**확인 방법**:
```bash
# IMU 데이터 확인
ros2 topic echo /imu/data --once
```

**해결방법**:
- `navsat_transform.yaml`의 `yaw_offset` 조정
- 자기 편각(magnetic declination) 설정 확인
- IMU 캘리브레이션 수행

## ⚙️ 고급 설정

### 센서 노이즈 튜닝

EKF 설정 파일에서 각 센서의 신뢰도를 조정할 수 있습니다:

```yaml
# 높은 값 = 낮은 신뢰도, 낮은 값 = 높은 신뢰도
process_noise_covariance: [
  0.05, 0, 0, ...  # X 위치 노이즈
  0, 0.05, 0, ...  # Y 위치 노이즈
  # ... 15x15 행렬 ...
]
```

### 다중 GPS 안테나

여러 GPS 안테나를 사용하는 경우:

```yaml
# ekf_global.yaml에 추가
odom2: /gps/fix2_transformed
odom2_config: [true, true, false, ...]
```

### 실시간 성능 최적화

```yaml
# 실시간 시스템용 설정
ekf_filter_node:
  ros__parameters:
    frequency: 50.0  # 높은 주파수
    predict_to_current_time: true
    publish_acceleration: false  # 불필요한 연산 제거
```

## 📊 성능 모니터링

### 실시간 상태 확인

```bash
# 위치 추정 정확도 모니터링
ros2 topic echo /diagnostics

# 각 EKF 노드 상태
ros2 node info /ekf_filter_local
ros2 node info /ekf_filter_global

# GPS 품질 확인
ros2 topic echo /gps/fix | grep -A3 "status"
```

### 로그 분석

```bash
# ROS 로그 위치
cd ~/.ros/log/

# GPS 변환 정확도 분석
ros2 topic echo /gps/filtered > gps_filtered.log
ros2 topic echo /gps/fix > gps_raw.log
```

## 🚀 Nav2와 통합

지도 보정이 완료되면 Nav2를 사용하여 완전한 자율주행을 구현할 수 있습니다:

```bash
# AMCL 없이 Nav2 실행
ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=false \
    params_file:=./config/nav2_params.yaml
```

**주의사항**: AMCL을 비활성화하고 우리의 EKF 시스템이 `map` → `odom` 변환을 제공하도록 해야 합니다.

## 📚 참고 자료

- [ROS2 robot_localization 문서](http://docs.ros.org/en/melodic/api/robot_localization/html/)
- [Nav2 GPS 튜토리얼](https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html)
- [REP-105: 좌표계 표준](https://www.ros.org/reps/rep-0105.html)

## 🤝 기여하기

이 프로젝트에 기여하고 싶다면:
1. 이슈 보고: [GitHub Issues](https://github.com/limchanggeon/ros2-delivery-robot/issues)
2. 풀 리퀘스트 환영
3. 문서 개선 제안

---

**⚡ 빠른 도움말**: 문제가 발생하면 `./scripts/quick_start_gps_system.sh --help` 실행 또는 이 문서의 문제 해결 섹션을 참조하세요!