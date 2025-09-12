# 강화학습 기반 RoboFuzz

RoboFuzz에 강화학습을 적용하여 더 효과적인 버그 유발 미션을 자동 생성하는 시스템입니다.

## 🚀 빠른 시작

### 1. 의존성 설치
```bash
cd /robofuzz
./install_rl_dependencies.sh
```

### 2. 기본 실행
```bash
# 강화학습 모델 훈련
python3 src/train_rl_model.py --episodes 1000

# RL 기반 퍼징 실행
python3 src/run_rl_fuzzing.py --px4-sitl-ros --cycles 100
```

## 📁 파일 구조

```
robofuzz/
├── src/
│   ├── rl_mission_generator.py      # DQN 기반 미션 생성기
│   ├── rl_fuzzer_integration.py     # RoboFuzz 통합 모듈
│   ├── run_rl_fuzzing.py           # 실행 스크립트
│   ├── train_rl_model.py           # 모델 훈련 스크립트
│   └── rl_config.py                # 설정 파일
├── missions/                        # 생성된 미션 파일들
├── models/                         # 훈련된 모델들
├── logs/                          # 로그 파일들
└── requirements_rl.txt             # 의존성 목록
```

## 🧠 강화학습 알고리즘

### DQN (Deep Q-Network)
- **상태 공간**: PX4 시스템 상태 (15차원)
- **행동 공간**: 웨이포인트 생성 (13차원)
- **보상 함수**: 버그 발견, 커버리지 증가, 시스템 안정성

### 핵심 구성요소
- **환경**: PX4 SITL + RoboFuzz
- **에이전트**: DQN 네트워크
- **경험 리플레이**: 과거 경험 학습
- **타겟 네트워크**: 안정적 학습

## ⚙️ 설정

### 기본 설정
```python
# rl_config.py에서 수정 가능
learning_rate = 0.001
gamma = 0.95
epsilon = 1.0
batch_size = 32
memory_size = 10000
```

### 환경별 설정
```bash
# 개발 환경 (빠른 테스트)
python3 train_rl_model.py --env development --episodes 100

# 프로덕션 환경 (완전한 훈련)
python3 train_rl_model.py --env production --episodes 10000
```

## 📊 사용법

### 1. 모델 훈련
```bash
# 기본 훈련
python3 src/train_rl_model.py --episodes 1000

# 개발용 훈련 (빠름)
python3 src/train_rl_model.py --env development --episodes 100
```

### 2. 퍼징 실행
```bash
# RL 전용 퍼징
python3 src/run_rl_fuzzing.py --px4-sitl-ros --mission-type rl --cycles 200

# 혼합 퍼징 (RL + 정적)
python3 src/run_rl_fuzzing.py --px4-sitl-ros --mission-type mixed --rl-ratio 0.5

# 고급 설정
python3 src/run_rl_fuzzing.py \
    --px4-sitl-ros \
    --rl-enabled \
    --rl-ratio 0.3 \
    --min-waypoints 30 \
    --max-waypoints 80 \
    --cycles 500
```

### 3. 모델 관리
```bash
# 모델 저장
python3 src/train_rl_model.py --episodes 1000

# 모델 로드
python3 src/run_rl_fuzzing.py --load-model models/rl_model_episode_1000.pth
```

## 📈 성능 모니터링

### 훈련 통계
- **에피소드 보상**: 평균 보상 추이
- **커버리지**: 코드 커버리지 증가량
- **버그 발견**: 발견된 버그 수
- **Epsilon**: 탐험률 변화

### 로그 파일
- `logs/training_stats.json`: 훈련 통계
- `logs/rl_fuzzing.log`: 퍼징 로그

## 🔧 문제 해결

### 의존성 문제
```bash
# PyTorch 재설치
pip3 uninstall torch
pip3 install torch --index-url https://download.pytorch.org/whl/cpu

# ROS2 문제
sudo apt update
sudo apt install ros-foxy-rclpy
```

### 메모리 부족
```bash
# 배치 크기 줄이기
python3 train_rl_model.py --episodes 1000 --batch-size 16
```

## 🎯 기대 효과

### 기존 RoboFuzz 대비
- **버그 발견률**: 30-50% 향상 예상
- **커버리지 효율성**: 20-40% 향상 예상
- **자동화**: 수동 미션 생성 불필요
- **적응성**: 학습을 통한 지속적 개선

### 발견 가능한 버그 유형
- **수치 오류**: NaN, INF, 오버플로우
- **물리적 한계**: 가속도, 속도, 추력 한계 초과
- **시스템 충격**: 급격한 변화로 인한 불안정
- **경계 조건**: 극값에서의 예외 상황

## 📚 참고 자료

- [RoboFuzz 원본 논문](링크)
- [DQN 알고리즘](https://arxiv.org/abs/1312.5602)
- [PX4 오프보드 모드](https://docs.px4.io/master/en/flight_modes/offboard.html)
