#!/bin/bash

# GPS 기반 자율주행 시스템 빠른 시작 스크립트
# 사용법: ./scripts/quick_start_gps_system.sh /path/to/your/map.yaml

set -e  # 에러 발생 시 스크립트 중단

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# 프로젝트 루트 디렉토리 설정
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &> /dev/null && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo -e "${BLUE}==============================================================${NC}"
echo -e "${BLUE}🚀 GPS 기반 자율주행 시스템 시작${NC}"
echo -e "${BLUE}==============================================================${NC}"

# 지도 파일 경로 확인
if [ $# -eq 0 ]; then
    echo -e "${RED}❌ 에러: 지도 파일 경로가 필요합니다${NC}"
    echo "사용법: $0 /path/to/your/map.yaml"
    echo ""
    echo "예시:"
    echo "  $0 $PROJECT_ROOT/maps/my_map.yaml"
    exit 1
fi

MAP_FILE="$1"

# 지도 파일 존재 확인
if [ ! -f "$MAP_FILE" ]; then
    echo -e "${RED}❌ 지도 파일을 찾을 수 없습니다: $MAP_FILE${NC}"
    exit 1
fi

echo -e "${GREEN}✅ 지도 파일 확인됨: $MAP_FILE${NC}"

# ROS2 환경 소싱
echo -e "${YELLOW}🔧 ROS2 환경 설정 중...${NC}"
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo -e "${GREEN}✅ ROS2 Humble 환경 로드됨${NC}"
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
    echo -e "${GREEN}✅ ROS2 Foxy 환경 로드됨${NC}"
else
    echo -e "${RED}❌ ROS2 환경을 찾을 수 없습니다${NC}"
    exit 1
fi

# 로컬 워크스페이스가 있다면 소싱
if [ -f "$PROJECT_ROOT/install/setup.bash" ]; then
    source "$PROJECT_ROOT/install/setup.bash"
    echo -e "${GREEN}✅ 로컬 워크스페이스 환경 로드됨${NC}"
fi

# 필수 패키지 확인
echo -e "${YELLOW}📦 필수 패키지 확인 중...${NC}"

required_packages=(
    "robot_localization"
    "nav2_map_server" 
    "nav2_lifecycle_manager"
    "tf2_tools"
    "rviz2"
)

missing_packages=()

for package in "${required_packages[@]}"; do
    if ! ros2 pkg list | grep -q "^$package$"; then
        missing_packages+=("$package")
    fi
done

if [ ${#missing_packages[@]} -ne 0 ]; then
    echo -e "${RED}❌ 다음 패키지들이 설치되지 않았습니다:${NC}"
    for package in "${missing_packages[@]}"; do
        echo -e "   • $package"
    done
    echo ""
    echo "다음 명령어로 설치하세요:"
    echo "sudo apt update"
    echo "sudo apt install ros-\$ROS_DISTRO-robot-localization ros-\$ROS_DISTRO-nav2-map-server ros-\$ROS_DISTRO-nav2-lifecycle-manager ros-\$ROS_DISTRO-tf2-tools ros-\$ROS_DISTRO-rviz2"
    exit 1
fi

echo -e "${GREEN}✅ 모든 필수 패키지 설치 확인됨${NC}"

# 설정 파일 확인
echo -e "${YELLOW}⚙️ 설정 파일 확인 중...${NC}"

config_files=(
    "$PROJECT_ROOT/config/ekf_local.yaml"
    "$PROJECT_ROOT/config/ekf_global.yaml"
    "$PROJECT_ROOT/config/navsat_transform.yaml"
    "$PROJECT_ROOT/config/gps_calibration_params.yaml"
)

for config_file in "${config_files[@]}"; do
    if [ ! -f "$config_file" ]; then
        echo -e "${RED}❌ 설정 파일을 찾을 수 없습니다: $config_file${NC}"
        exit 1
    fi
done

echo -e "${GREEN}✅ 모든 설정 파일 확인됨${NC}"

# 센서 토픽 확인 함수
check_topic() {
    local topic=$1
    local timeout=5
    
    echo -e "${CYAN}🔍 토픽 확인 중: $topic${NC}"
    
    if timeout $timeout ros2 topic info "$topic" >/dev/null 2>&1; then
        echo -e "${GREEN}✅ $topic 토픽 활성화됨${NC}"
        return 0
    else
        echo -e "${YELLOW}⚠️  $topic 토픽을 찾을 수 없습니다${NC}"
        return 1
    fi
}

# 센서 토픽들 확인
echo -e "${YELLOW}📡 센서 토픽 상태 확인 중...${NC}"

sensor_topics=(
    "/gps/fix"
    "/imu/data"
    "/odom"
)

missing_topics=()

for topic in "${sensor_topics[@]}"; do
    if ! check_topic "$topic"; then
        missing_topics+=("$topic")
    fi
done

if [ ${#missing_topics[@]} -ne 0 ]; then
    echo -e "${YELLOW}⚠️  다음 센서 토픽들이 아직 활성화되지 않았습니다:${NC}"
    for topic in "${missing_topics[@]}"; do
        echo -e "   • $topic"
    done
    echo -e "${YELLOW}센서 노드들을 먼저 실행하거나, 계속 진행하려면 Enter를 누르세요...${NC}"
    read -r
fi

# 시스템 시작
echo -e "${PURPLE}🎯 GPS 자율주행 시스템 시작 중...${NC}"

# 런치 명령어 생성
LAUNCH_CMD="ros2 launch $PROJECT_ROOT/launch/gps_autonomous_system.launch.py"
LAUNCH_CMD="$LAUNCH_CMD map_file:=$MAP_FILE"
LAUNCH_CMD="$LAUNCH_CMD use_sim_time:=false"
LAUNCH_CMD="$LAUNCH_CMD enable_calibration:=true"

echo -e "${CYAN}실행 명령어:${NC}"
echo "$LAUNCH_CMD"
echo ""

# 실행 전 최종 확인
echo -e "${YELLOW}🚨 시스템을 시작하시겠습니까? (y/N)${NC}"
read -r response

if [[ "$response" =~ ^([yY][eE][sS]|[yY])$ ]]; then
    echo -e "${GREEN}🎉 시스템 시작!${NC}"
    echo ""
    
    # 별도 터미널에서 RViz 실행 (백그라운드)
    echo -e "${CYAN}📊 RViz2 시작 중...${NC}"
    gnome-terminal -- bash -c "sleep 5; ros2 run rviz2 rviz2; exec bash" 2>/dev/null || \
    osascript -e 'tell app "Terminal" to do script "sleep 5; ros2 run rviz2 rviz2"' 2>/dev/null || \
    echo -e "${YELLOW}⚠️  RViz 자동 실행 실패. 수동으로 실행하세요: ros2 run rviz2 rviz2${NC}"
    
    # 메인 시스템 실행
    eval $LAUNCH_CMD
    
else
    echo -e "${YELLOW}시스템 시작이 취소되었습니다.${NC}"
fi

echo -e "${BLUE}==============================================================${NC}"
echo -e "${BLUE}📖 도움말 및 문제 해결${NC}"
echo -e "${BLUE}==============================================================${NC}"
echo -e "🔧 문제 해결 명령어들:"
echo -e "  • TF 트리 확인: ${CYAN}ros2 run tf2_tools view_frames${NC}"
echo -e "  • 토픽 목록: ${CYAN}ros2 topic list${NC}"
echo -e "  • 노드 상태: ${CYAN}ros2 node list${NC}"
echo -e "  • GPS 상태: ${CYAN}ros2 topic echo /gps/fix${NC}"
echo ""
echo -e "🗺️ 지도 보정 방법:"
echo -e "  1. RViz에서 Fixed Frame을 'map'으로 설정"
echo -e "  2. Map display 추가"
echo -e "  3. 'Publish Point' 도구로 지도 위 클릭"
echo -e "  4. GPS 보정 완료!"