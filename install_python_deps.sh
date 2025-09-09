#!/bin/bash

# Python 의존성 설치 스크립트 (메모리 효율적 설치)
# 작성자: 배달로봇팀
# 날짜: 2025-09-09

set -e

echo "========================================="
echo "Python 의존성 설치 시작"
echo "========================================="

# pip 업그레이드
echo "📦 pip 업그레이드 중..."
python3 -m pip install --upgrade pip

# 메모리 효율적 설치를 위한 함수
install_with_retry() {
    local package_name=$1
    shift  # 첫 번째 인자 제거
    local pip_args="$@"  # 나머지 모든 인자들
    local max_retries=3
    local retry=0
    
    while [ $retry -lt $max_retries ]; do
        echo "설치 시도 ($((retry+1))/$max_retries): $package_name"
        if [ -n "$pip_args" ]; then
            # 추가 pip 인자가 있는 경우
            if python3 -m pip install --no-cache-dir --user $pip_args; then
                echo "✅ $package_name 설치 성공"
                return 0
            fi
        else
            # 단순 패키지 설치
            if python3 -m pip install --no-cache-dir --user "$package_name"; then
                echo "✅ $package_name 설치 성공"
                return 0
            fi
        fi
        echo "❌ $package_name 설치 실패, 재시도 중..."
        retry=$((retry+1))
        sleep 2
    done
    
    echo "❌ $package_name 설치 최종 실패"
    return 1
}

# 기본 패키지들 먼저 설치
echo "🔧 기본 패키지 설치 중..."
install_with_retry "numpy"
install_with_retry "opencv-python"

# PyTorch 설치 (젯슨용 CUDA 버전)
echo "🤖 PyTorch 설치 중..."
if [ -f "/proc/device-tree/model" ] && grep -qi "jetson" /proc/device-tree/model 2>/dev/null; then
    echo "젯슨 디바이스 감지됨 - CUDA 지원 PyTorch 설치"
    # 젯슨용 PyTorch wheel 다운로드 및 설치
    TORCH_WHEEL="torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl"
    if [ ! -f "$TORCH_WHEEL" ]; then
        echo "젯슨용 PyTorch wheel 다운로드 중..."
        wget -q https://developer.download.nvidia.com/compute/redist/jp/v50/pytorch/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl
    fi
    install_with_retry "젯슨용 PyTorch" "$TORCH_WHEEL"
    
    # torchvision 설치 (젯슨 호환)
    install_with_retry "torchvision==0.15.1"
else
    echo "일반 시스템 - CPU PyTorch 설치"
    install_with_retry "CPU PyTorch" torch torchvision --index-url https://download.pytorch.org/whl/cpu
fi

# YOLOv8 및 관련 패키지
echo "👁️ YOLOv8 및 컴퓨터 비전 패키지 설치 중..."
install_with_retry "ultralytics"

# 웹 및 통신 패키지
echo "🌐 웹 및 통신 패키지 설치 중..."
install_with_retry "requests"
install_with_retry "geopy"

# 시리얼 통신
echo "📡 시리얼 통신 패키지 설치 중..."
install_with_retry "pyserial"

# 보안 및 암호화
echo "🔒 보안 패키지 설치 중..."
install_with_retry "cryptography"

# QR 코드 관련
echo "📱 QR 코드 패키지 설치 중..."
install_with_retry "qrcode[pil]"
install_with_retry "pyzbar"

# 추가 유틸리티
echo "🛠️ 추가 유틸리티 설치 중..."
install_with_retry "pillow"
install_with_retry "matplotlib"

echo ""
echo "========================================="
echo "✅ Python 의존성 설치 완료!"
echo "========================================="

# 설치된 패키지 확인
echo "📋 설치된 주요 패키지 버전:"
python3 -c "
import sys
packages = ['numpy', 'cv2', 'torch', 'ultralytics', 'requests', 'geopy', 'serial', 'cryptography', 'qrcode', 'pyzbar']
for pkg in packages:
    try:
        if pkg == 'cv2':
            import cv2
            print(f'✅ opencv-python: {cv2.__version__}')
        elif pkg == 'serial':
            import serial
            print(f'✅ pyserial: {serial.__version__}')
        else:
            module = __import__(pkg)
            version = getattr(module, '__version__', 'unknown')
            print(f'✅ {pkg}: {version}')
    except ImportError:
        print(f'❌ {pkg}: Not installed')
"

echo ""
echo "🎉 모든 Python 의존성이 성공적으로 설치되었습니다!"