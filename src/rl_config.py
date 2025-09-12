#!/usr/bin/env python3
"""
강화학습 기반 RoboFuzz 설정 파일
"""

import os
from dataclasses import dataclass
from typing import List, Dict, Any

@dataclass
class RLConfig:
    """강화학습 설정"""
    
    # DQN 하이퍼파라미터
    learning_rate: float = 0.001
    gamma: float = 0.95
    epsilon: float = 1.0
    epsilon_min: float = 0.01
    epsilon_decay: float = 0.995
    batch_size: int = 32
    memory_size: int = 10000
    target_update_freq: int = 100
    
    # 네트워크 구조
    state_size: int = 15
    action_size: int = 13
    hidden_size: int = 128
    
    # 미션 생성 설정
    min_waypoints: int = 20
    max_waypoints: int = 100
    rl_mission_ratio: float = 0.3
    
    # 학습 설정
    max_episodes: int = 1000
    save_freq: int = 100
    log_freq: int = 10
    
    # 파일 경로
    model_save_path: str = "/robofuzz/src/models"
    log_path: str = "/robofuzz/src/logs"
    mission_path: str = "/robofuzz/src/missions"
    
    # PX4 설정
    px4_config: Dict[str, Any] = None
    
    def __post_init__(self):
        """초기화 후 처리"""
        # 디렉토리 생성
        os.makedirs(self.model_save_path, exist_ok=True)
        os.makedirs(self.log_path, exist_ok=True)
        os.makedirs(self.mission_path, exist_ok=True)
        
        # PX4 기본 설정
        if self.px4_config is None:
            self.px4_config = {
                'position_range': {'x': [-100, 100], 'y': [-100, 100], 'z': [-20, -1]},
                'yaw_range': [-3.14, 3.14],
                'acceleration_range': [-5000, 5000],
                'velocity_range': [-10, 10],
                'jerk_range': [-1000, 1000],
                'thrust_range': [-1000, 1000]
            }

# 전역 설정 인스턴스
rl_config = RLConfig()

# 환경별 설정
def get_config(env: str = "default") -> RLConfig:
    """환경별 설정 반환"""
    if env == "development":
        config = RLConfig()
        config.learning_rate = 0.01
        config.epsilon_decay = 0.99
        config.max_episodes = 100
        return config
    elif env == "production":
        config = RLConfig()
        config.learning_rate = 0.0001
        config.epsilon_decay = 0.999
        config.max_episodes = 10000
        return config
    else:
        return rl_config
