#!/bin/bash
# 강화학습 기반 RoboFuzz 의존성 설치 스크립트

echo "=== 강화학습 기반 RoboFuzz 의존성 설치 ==="

# Python 패키지 관리자 확인
if command -v pip3 &> /dev/null; then
    PIP_CMD="pip3"
elif command -v pip &> /dev/null; then
    PIP_CMD="pip"
else
    echo "❌ pip이 설치되어 있지 않습니다."
    exit 1
fi

echo "사용할 pip: $PIP_CMD"

# 기본 의존성 설치
echo "📦 기본 의존성 설치 중..."
$PIP_CMD install --upgrade pip
$PIP_CMD install numpy scipy matplotlib

# PyTorch 설치 (CPU 버전)
echo "🔥 PyTorch 설치 중..."
$PIP_CMD install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu

# ROS2 의존성 (Ubuntu 20.04 기준)
echo "🤖 ROS2 의존성 설치 중..."
if ! dpkg -l | grep -q ros-foxy; then
    echo "ROS2 Foxy가 설치되어 있지 않습니다. 설치를 진행합니다..."
    # ROS2 설치 (간단한 버전)
    sudo apt update
    sudo apt install -y python3-colcon-common-extensions python3-rosdep
else
    echo "✅ ROS2가 이미 설치되어 있습니다."
fi

# rclpy 설치
echo "📡 rclpy 설치 중..."
$PIP_CMD install rclpy

# 추가 ML 라이브러리
echo "🧠 추가 ML 라이브러리 설치 중..."
$PIP_CMD install stable-baselines3 gym tensorboard

# 설치 확인
echo "✅ 설치 확인 중..."
python3 -c "
import torch
import numpy as np
import rclpy
print('✅ 모든 의존성이 성공적으로 설치되었습니다!')
print(f'PyTorch 버전: {torch.__version__}')
print(f'NumPy 버전: {np.__version__}')
print(f'RCLpy 사용 가능: {rclpy is not None}')
"

echo "🎉 설치 완료!"
echo ""
echo "사용 방법:"
echo "1. 훈련: python3 train_rl_model.py --episodes 1000"
echo "2. 실행: python3 run_rl_fuzzing.py --px4-sitl-ros"
