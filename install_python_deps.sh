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
    local package=$1
    local max_retries=3
    local retry=0
    
    while [ $retry -lt $max_retries ]; do
        echo "설치 시도 ($((retry+1))/$max_retries): $package"
        if python3 -m pip install --no-cache-dir --user "$package"; then
            echo "✅ $package 설치 성공"
            return 0
        else
            echo "❌ $package 설치 실패, 재시도 중..."
            retry=$((retry+1))
            sleep 2
        fi
    done
    
    echo "❌ $package 설치 최종 실패"
    return 1
}

# 기본 패키지들 먼저 설치
echo "🔧 기본 패키지 설치 중..."
install_with_retry "numpy"
install_with_retry "opencv-python"

# PyTorch 설치 (CPU 버전, 더 가벼움)
echo "🤖 PyTorch 설치 중..."
install_with_retry "torch torchvision --index-url https://download.pytorch.org/whl/cpu"

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