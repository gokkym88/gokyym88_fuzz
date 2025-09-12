#!/usr/bin/env python3
"""
강화학습 기반 동적 미션 생성기
RoboFuzz와 통합하여 더 효과적인 버그 유발 경로를 생성
"""

import json
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import random
import math
from collections import deque
from typing import List, Dict, Tuple, Optional
import rclpy
from rclpy.node import Node

class MissionState:
    """미션 상태를 나타내는 클래스"""
    def __init__(self):
        self.position = [0.0, 0.0, -5.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.attitude = [0.0, 0.0, 0.0]
        self.acceleration = [0.0, 0.0, 0.0]
        self.coverage_map = []
        self.new_coverage = []
        self.path_history = []
        self.bug_history = []
        self.system_health = {}
        self.oracle_violations = []
        
    def to_tensor(self) -> torch.Tensor:
        """상태를 텐서로 변환"""
        state_vector = (
            self.position + 
            self.velocity + 
            self.attitude + 
            self.acceleration +
            [len(self.coverage_map)] +
            [len(self.new_coverage)] +
            [len(self.bug_history)]
        )
        return torch.FloatTensor(state_vector)

class MissionAction:
    """미션 행동을 나타내는 클래스"""
    def __init__(self, x=0.0, y=0.0, z=-5.0, yaw=0.0, 
                 acceleration=[0.0, 0.0, 0.0], 
                 velocity=[0.0, 0.0, 0.0],
                 jerk=[0.0, 0.0, 0.0],
                 thrust=[0.0, 0.0, 0.0]):
        self.waypoint = {
            'timestamp': 0,
            'x': x, 'y': y, 'z': z,
            'yaw': yaw, 'yawspeed': 0.0,
            'vx': velocity[0], 'vy': velocity[1], 'vz': velocity[2],
            'acceleration': acceleration,
            'jerk': jerk,
            'thrust': thrust
        }
        
    def to_tensor(self) -> torch.Tensor:
        """행동을 텐서로 변환"""
        action_vector = [
            self.waypoint['x'], self.waypoint['y'], self.waypoint['z'],
            self.waypoint['yaw'],
            *self.waypoint['acceleration'],
            *self.waypoint['velocity'],
            *self.waypoint['jerk'],
            *self.waypoint['thrust']
        ]
        return torch.FloatTensor(action_vector)

class DQN(nn.Module):
    """Deep Q-Network for mission generation"""
    def __init__(self, state_size=15, action_size=13, hidden_size=128):
        super(DQN, self).__init__()
        self.fc1 = nn.Linear(state_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.fc3 = nn.Linear(hidden_size, hidden_size)
        self.fc4 = nn.Linear(hidden_size, action_size)
        self.dropout = nn.Dropout(0.2)
        
    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = self.dropout(x)
        x = torch.relu(self.fc2(x))
        x = self.dropout(x)
        x = torch.relu(self.fc3(x))
        x = self.fc4(x)
        return x

class ReplayBuffer:
    """경험 리플레이 버퍼"""
    def __init__(self, capacity=10000):
        self.buffer = deque(maxlen=capacity)
        
    def push(self, state, action, reward, next_state, done):
        self.buffer.append((state, action, reward, next_state, done))
        
    def sample(self, batch_size):
        batch = random.sample(self.buffer, batch_size)
        state, action, reward, next_state, done = map(np.stack, zip(*batch))
        return state, action, reward, next_state, done
        
    def __len__(self):
        return len(self.buffer)

class RLMissionGenerator(Node):
    """강화학습 기반 미션 생성기"""
    
    def __init__(self):
        super().__init__('rl_mission_generator')
        
        # DQN 하이퍼파라미터
        self.state_size = 15
        self.action_size = 13
        self.lr = 0.001
        self.gamma = 0.95
        self.epsilon = 1.0
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.995
        self.batch_size = 32
        self.memory_size = 10000
        
        # DQN 네트워크
        self.q_network = DQN(self.state_size, self.action_size)
        self.target_network = DQN(self.state_size, self.action_size)
        self.optimizer = optim.Adam(self.q_network.parameters(), lr=self.lr)
        self.memory = ReplayBuffer(self.memory_size)
        
        # 학습 통계
        self.episode_rewards = []
        self.episode_coverage = []
        self.episode_bugs = []
        
        # 미션 생성 통계
        self.generated_missions = 0
        self.successful_bugs = 0
        
    def calculate_reward(self, state: MissionState, action: MissionAction, 
                        next_state: MissionState, feedback: Dict) -> float:
        """보상 함수 계산"""
        reward = 0.0
        
        # 1. 커버리지 증가 보상
        new_coverage = len(next_state.new_coverage)
        reward += new_coverage * 10.0
        
        # 2. 버그 발견 보상
        if feedback.get('crash_detected', False):
            reward += 100.0
            self.successful_bugs += 1
        elif feedback.get('oracle_violation', False):
            reward += 50.0
        elif feedback.get('interesting_case', False):
            reward += 20.0
        
        # 3. 새로운 패턴 발견 보상
        if feedback.get('new_pattern', False):
            reward += 30.0
        
        # 4. 시스템 안정성 페널티
        if feedback.get('system_unstable', False):
            reward -= 10.0
        
        # 5. 효율성 보상
        if len(action.waypoint) > 0:
            efficiency = new_coverage / 1.0  # 단일 웨이포인트 기준
            reward += efficiency * 5.0
        
        return reward
    
    def select_action(self, state: MissionState, training=True) -> MissionAction:
        """ε-greedy 정책으로 행동 선택"""
        if training and random.random() < self.epsilon:
            # 랜덤 행동 (탐험)
            return self._random_action()
        else:
            # DQN으로 행동 선택 (활용)
            return self._greedy_action(state)
    
    def _random_action(self) -> MissionAction:
        """랜덤 행동 생성"""
        x = random.uniform(-100, 100)
        y = random.uniform(-100, 100)
        z = random.uniform(-20, -1)
        yaw = random.uniform(-math.pi, math.pi)
        
        # 스트레스 테스트를 위한 비정상 값 생성
        if random.random() < 0.3:  # 30% 확률
            acceleration = [
                random.uniform(-5000, 5000),
                random.uniform(-5000, 5000),
                random.uniform(-5000, 5000)
            ]
        else:
            acceleration = [0.0, 0.0, 0.0]
            
        return MissionAction(x, y, z, yaw, acceleration)
    
    def _greedy_action(self, state: MissionState) -> MissionAction:
        """DQN으로 최적 행동 선택"""
        state_tensor = state.to_tensor().unsqueeze(0)
        q_values = self.q_network(state_tensor)
        
        # 연속 행동 공간을 이산화
        action_indices = q_values.argmax(dim=1)
        
        # 인덱스를 실제 행동으로 변환
        return self._index_to_action(action_indices[0].item())
    
    def _index_to_action(self, action_index: int) -> MissionAction:
        """행동 인덱스를 실제 행동으로 변환"""
        # 간단한 매핑 (실제로는 더 복잡한 변환 필요)
        x = (action_index % 20 - 10) * 10  # -100 ~ 100
        y = ((action_index // 20) % 20 - 10) * 10
        z = -random.uniform(1, 20)
        yaw = random.uniform(-math.pi, math.pi)
        
        return MissionAction(x, y, z, yaw)
    
    def train_step(self):
        """DQN 학습 스텝"""
        if len(self.memory) < self.batch_size:
            return
            
        # 배치 샘플링
        states, actions, rewards, next_states, dones = self.memory.sample(self.batch_size)
        
        # 현재 Q 값 계산
        current_q_values = self.q_network(states).gather(1, actions.unsqueeze(1))
        
        # 다음 상태의 최대 Q 값 계산
        next_q_values = self.target_network(next_states).max(1)[0].detach()
        target_q_values = rewards + (self.gamma * next_q_values * (1 - dones))
        
        # 손실 계산 및 역전파
        loss = nn.MSELoss()(current_q_values.squeeze(), target_q_values)
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
        
        # ε 감소
        if self.epsilon > self.epsilon_min:
            self.epsilon *= self.epsilon_decay
    
    def update_target_network(self):
        """타겟 네트워크 업데이트"""
        self.target_network.load_state_dict(self.q_network.state_dict())
    
    def generate_mission(self, num_waypoints: int = 30, 
                        target_bugs: List[str] = None) -> str:
        """동적 미션 생성"""
        mission_waypoints = []
        current_state = MissionState()
        
        for i in range(num_waypoints):
            # 행동 선택
            action = self.select_action(current_state, training=False)
            mission_waypoints.append(action.waypoint)
            
            # 상태 업데이트 (시뮬레이션)
            current_state.position = [action.waypoint['x'], 
                                    action.waypoint['y'], 
                                    action.waypoint['z']]
            current_state.attitude[2] = action.waypoint['yaw']
            current_state.acceleration = action.waypoint['acceleration']
            current_state.path_history.append(action.waypoint)
        
        # 미션 파일 저장
        mission_filename = f"rl_mission_{self.generated_missions}.json"
        mission_path = f"/robofuzz/src/missions/{mission_filename}"
        
        with open(mission_path, 'w') as f:
            json.dump(mission_waypoints, f, separators=(',', ':'))
        
        self.generated_missions += 1
        self.get_logger().info(f"Generated RL mission: {mission_filename}")
        
        return mission_path
    
    def learn_from_feedback(self, mission_path: str, feedback: Dict):
        """피드백으로부터 학습"""
        # 미션 파일 로드
        with open(mission_path, 'r') as f:
            waypoints = json.load(f)
        
        # 각 웨이포인트에 대해 학습
        for i, waypoint in enumerate(waypoints):
            state = MissionState()
            if i > 0:
                prev_waypoint = waypoints[i-1]
                state.position = [prev_waypoint['x'], prev_waypoint['y'], prev_waypoint['z']]
                state.attitude[2] = prev_waypoint['yaw']
                state.acceleration = prev_waypoint['acceleration']
            
            action = MissionAction(
                waypoint['x'], waypoint['y'], waypoint['z'],
                waypoint['yaw'], waypoint['acceleration'],
                [waypoint['vx'], waypoint['vy'], waypoint['vz']],
                waypoint['jerk'], waypoint['thrust']
            )
            
            next_state = MissionState()
            next_state.position = [waypoint['x'], waypoint['y'], waypoint['z']]
            next_state.attitude[2] = waypoint['yaw']
            next_state.acceleration = waypoint['acceleration']
            
            # 보상 계산
            reward = self.calculate_reward(state, action, next_state, feedback)
            
            # 메모리에 저장
            done = (i == len(waypoints) - 1)
            self.memory.push(
                state.to_tensor().numpy(),
                action.to_tensor().numpy(),
                reward,
                next_state.to_tensor().numpy(),
                done
            )
        
        # 학습 수행
        self.train_step()
        
        # 통계 업데이트
        self.episode_rewards.append(feedback.get('total_reward', 0))
        self.episode_coverage.append(feedback.get('coverage_gained', 0))
        self.episode_bugs.append(feedback.get('bugs_found', 0))
    
    def get_statistics(self) -> Dict:
        """학습 통계 반환"""
        return {
            'generated_missions': self.generated_missions,
            'successful_bugs': self.successful_bugs,
            'avg_reward': np.mean(self.episode_rewards[-100:]) if self.episode_rewards else 0,
            'avg_coverage': np.mean(self.episode_coverage[-100:]) if self.episode_coverage else 0,
            'avg_bugs': np.mean(self.episode_bugs[-100:]) if self.episode_bugs else 0,
            'epsilon': self.epsilon
        }

def main():
    """메인 함수"""
    rclpy.init()
    
    # RL 미션 생성기 초기화
    rl_generator = RLMissionGenerator()
    
    # 예제: 동적 미션 생성
    mission_path = rl_generator.generate_mission(num_waypoints=50)
    print(f"Generated mission: {mission_path}")
    
    # 예제: 피드백 학습
    feedback = {
        'crash_detected': False,
        'oracle_violation': True,
        'interesting_case': True,
        'new_pattern': False,
        'system_unstable': False,
        'total_reward': 75.0,
        'coverage_gained': 5,
        'bugs_found': 2
    }
    
    rl_generator.learn_from_feedback(mission_path, feedback)
    
    # 통계 출력
    stats = rl_generator.get_statistics()
    print(f"RL Statistics: {stats}")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
