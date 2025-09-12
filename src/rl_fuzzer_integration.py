#!/usr/bin/env python3
"""
RoboFuzz와 강화학습 미션 생성기의 통합 모듈
기존 fuzzer.py를 확장하여 RL 기반 동적 미션 생성 지원
"""

import os
import json
import time
import numpy as np
from typing import Dict, List, Optional
import rclpy
from rclpy.node import Node

# 기존 RoboFuzz 모듈들 import
from fuzzer import Fuzzer
from rl_mission_generator import RLMissionGenerator

class RLFuzzerIntegration(Node):
    """RoboFuzz와 RL 미션 생성기의 통합 클래스"""
    
    def __init__(self, config):
        super().__init__('rl_fuzzer_integration')
        
        # 기존 Fuzzer 초기화
        self.fuzzer = Fuzzer("_rl_fuzzer", config)
        self.fuzzer.init_cov_map()
        self.fuzzer.init_queue()
        
        # RL 미션 생성기 초기화
        self.rl_generator = RLMissionGenerator()
        
        # 통합 설정
        self.rl_enabled = getattr(config, 'rl_enabled', True)
        self.rl_learning_rate = getattr(config, 'rl_learning_rate', 0.1)
        self.rl_mission_ratio = getattr(config, 'rl_mission_ratio', 0.3)  # 30% RL 미션
        
        # 통계
        self.total_missions = 0
        self.rl_missions = 0
        self.static_missions = 0
        self.bugs_found = 0
        self.coverage_gained = 0
        
        # 피드백 히스토리
        self.feedback_history = []
        
    def generate_mission(self, mission_type: str = "mixed") -> str:
        """미션 생성 (RL 또는 정적)"""
        self.total_missions += 1
        
        if mission_type == "rl" or (mission_type == "mixed" and np.random.random() < self.rl_mission_ratio):
            # RL 기반 동적 미션 생성
            mission_path = self._generate_rl_mission()
            self.rl_missions += 1
            self.get_logger().info(f"Generated RL mission: {mission_path}")
        else:
            # 기존 정적 미션 사용
            mission_path = self._select_static_mission()
            self.static_missions += 1
            self.get_logger().info(f"Using static mission: {mission_path}")
        
        return mission_path
    
    def _generate_rl_mission(self) -> str:
        """RL 기반 미션 생성"""
        # 미션 파라미터 결정
        num_waypoints = np.random.randint(20, 100)
        target_bugs = self._get_target_bug_types()
        
        # RL 미션 생성
        mission_path = self.rl_generator.generate_mission(
            num_waypoints=num_waypoints,
            target_bugs=target_bugs
        )
        
        return mission_path
    
    def _select_static_mission(self) -> str:
        """기존 정적 미션 선택"""
        missions_dir = "/robofuzz/src/missions"
        static_missions = [
            "church-fail.json",
            "church.json", 
            "takeoff.json",
            "test.json"
        ]
        
        selected_mission = np.random.choice(static_missions)
        return os.path.join(missions_dir, selected_mission)
    
    def _get_target_bug_types(self) -> List[str]:
        """타겟 버그 유형 결정"""
        bug_types = [
            "acceleration_overflow",
            "gps_inconsistency", 
            "position_drift",
            "attitude_instability",
            "velocity_anomaly"
        ]
        
        # 현재 커버리지에 따라 타겟 버그 선택
        return np.random.choice(bug_types, size=np.random.randint(1, 4), replace=False).tolist()
    
    def run_fuzzing_cycle(self, mission_path: str) -> Dict:
        """단일 퍼징 사이클 실행"""
        # 미션 파일 설정
        self.fuzzer.config.px4_mission_file = mission_path
        
        # PX4 브리지 초기화 (필요시)
        if self.fuzzer.config.px4_sitl:
            self.fuzzer.init_px4_bridge()
        
        # 타겟 실행
        self.fuzzer.run_target(
            self.fuzzer.config.rospkg,
            self.fuzzer.config.rosnode, 
            self.fuzzer.config.exec_cmd
        )
        
        # 퍼징 실행
        feedback = self._execute_fuzzing()
        
        # 피드백 처리
        self._process_feedback(mission_path, feedback)
        
        return feedback
    
    def _execute_fuzzing(self) -> Dict:
        """실제 퍼징 실행 및 피드백 수집"""
        feedback = {
            'crash_detected': False,
            'oracle_violation': False,
            'interesting_case': False,
            'new_pattern': False,
            'system_unstable': False,
            'coverage_gained': 0,
            'bugs_found': 0,
            'total_reward': 0.0
        }
        
        try:
            # 기존 fuzzer의 fuzz_msg 함수 호출
            from fuzzer import fuzz_msg, inspect_target
            
            # 타겟 검사
            fuzz_targets = inspect_target(self.fuzzer)
            
            if fuzz_targets:
                # 퍼징 실행
                fuzz_msg(self.fuzzer, fuzz_targets)
                
                # 피드백 수집
                feedback = self._collect_feedback()
            
        except Exception as e:
            self.get_logger().error(f"Fuzzing execution failed: {e}")
            feedback['system_unstable'] = True
        
        return feedback
    
    def _collect_feedback(self) -> Dict:
        """피드백 수집"""
        feedback = {
            'crash_detected': False,
            'oracle_violation': False,
            'interesting_case': False,
            'new_pattern': False,
            'system_unstable': False,
            'coverage_gained': 0,
            'bugs_found': 0,
            'total_reward': 0.0
        }
        
        # 커버리지 변화 확인
        if hasattr(self.fuzzer, 'coverage_map'):
            current_coverage = sum(self.fuzzer.coverage_map)
            feedback['coverage_gained'] = max(0, current_coverage - getattr(self, 'last_coverage', 0))
            self.last_coverage = current_coverage
        
        # 오라클 위반 확인 (PX4 오라클 사용)
        if self.fuzzer.config.px4_sitl:
            try:
                from oracles.px4 import check
                # 오라클 체크 로직 (실제 구현 필요)
                feedback['oracle_violation'] = self._check_px4_oracle()
            except ImportError:
                pass
        
        # 크래시 감지
        feedback['crash_detected'] = self._detect_crash()
        
        # 흥미로운 케이스 감지
        feedback['interesting_case'] = self._detect_interesting_case()
        
        # 버그 수 계산
        feedback['bugs_found'] = sum([
            feedback['crash_detected'],
            feedback['oracle_violation'],
            feedback['interesting_case']
        ])
        
        # 총 보상 계산
        feedback['total_reward'] = self._calculate_reward(feedback)
        
        return feedback
    
    def _check_px4_oracle(self) -> bool:
        """PX4 오라클 위반 확인"""
        # 실제 구현에서는 PX4 상태를 모니터링
        # 여기서는 간단한 시뮬레이션
        return np.random.random() < 0.1  # 10% 확률로 위반
    
    def _detect_crash(self) -> bool:
        """크래시 감지"""
        # 실제 구현에서는 시스템 상태를 모니터링
        return np.random.random() < 0.05  # 5% 확률로 크래시
    
    def _detect_interesting_case(self) -> bool:
        """흥미로운 케이스 감지"""
        # 실제 구현에서는 커버리지 변화 등을 확인
        return np.random.random() < 0.2  # 20% 확률로 흥미로운 케이스
    
    def _calculate_reward(self, feedback: Dict) -> float:
        """보상 계산"""
        reward = 0.0
        reward += feedback['coverage_gained'] * 10.0
        reward += feedback['bugs_found'] * 50.0
        if feedback['crash_detected']:
            reward += 100.0
        if feedback['system_unstable']:
            reward -= 10.0
        return reward
    
    def _process_feedback(self, mission_path: str, feedback: Dict):
        """피드백 처리 및 학습"""
        # 피드백 히스토리에 저장
        self.feedback_history.append({
            'mission_path': mission_path,
            'feedback': feedback,
            'timestamp': time.time()
        })
        
        # RL 학습 (RL 미션인 경우)
        if 'rl_mission' in mission_path:
            self.rl_generator.learn_from_feedback(mission_path, feedback)
        
        # 통계 업데이트
        self.bugs_found += feedback['bugs_found']
        self.coverage_gained += feedback['coverage_gained']
        
        # 로깅
        self.get_logger().info(
            f"Feedback: bugs={feedback['bugs_found']}, "
            f"coverage={feedback['coverage_gained']}, "
            f"reward={feedback['total_reward']:.2f}"
        )
    
    def run_adaptive_fuzzing(self, num_cycles: int = 100):
        """적응형 퍼징 실행"""
        self.get_logger().info(f"Starting adaptive fuzzing for {num_cycles} cycles")
        
        for cycle in range(num_cycles):
            # 미션 생성
            mission_path = self.generate_mission(mission_type="mixed")
            
            # 퍼징 사이클 실행
            feedback = self.run_fuzzing_cycle(mission_path)
            
            # 주기적 통계 출력
            if cycle % 10 == 0:
                self._print_statistics()
            
            # RL 네트워크 업데이트 (주기적)
            if cycle % 50 == 0 and self.rl_enabled:
                self.rl_generator.update_target_network()
        
        # 최종 통계
        self._print_final_statistics()
    
    def _print_statistics(self):
        """통계 출력"""
        rl_stats = self.rl_generator.get_statistics()
        
        self.get_logger().info(
            f"Cycle Statistics: "
            f"Total={self.total_missions}, "
            f"RL={self.rl_missions}, "
            f"Static={self.static_missions}, "
            f"Bugs={self.bugs_found}, "
            f"Coverage={self.coverage_gained}, "
            f"RL_Epsilon={rl_stats['epsilon']:.3f}"
        )
    
    def _print_final_statistics(self):
        """최종 통계 출력"""
        rl_stats = self.rl_generator.get_statistics()
        
        self.get_logger().info("=== Final Statistics ===")
        self.get_logger().info(f"Total missions: {self.total_missions}")
        self.get_logger().info(f"RL missions: {self.rl_missions} ({self.rl_missions/self.total_missions*100:.1f}%)")
        self.get_logger().info(f"Static missions: {self.static_missions}")
        self.get_logger().info(f"Total bugs found: {self.bugs_found}")
        self.get_logger().info(f"Total coverage gained: {self.coverage_gained}")
        self.get_logger().info(f"RL success rate: {rl_stats['successful_bugs']/max(1, self.rl_missions)*100:.1f}%")
        self.get_logger().info(f"Average RL reward: {rl_stats['avg_reward']:.2f}")

def main():
    """메인 함수"""
    # 설정 초기화
    class Config:
        def __init__(self):
            self.px4_sitl = True
            self.px4_ros = True
            self.use_mavlink = False
            self.exp_pgfuzz = False
            self.rospkg = "px4"
            self.rosnode = "px4"
            self.exec_cmd = "px4"
            self.rl_enabled = True
            self.rl_learning_rate = 0.1
            self.rl_mission_ratio = 0.3
    
    config = Config()
    
    # RCL 초기화
    rclpy.init()
    
    # RL Fuzzer 통합 실행
    rl_fuzzer = RLFuzzerIntegration(config)
    
    try:
        # 적응형 퍼징 실행
        rl_fuzzer.run_adaptive_fuzzing(num_cycles=50)
    except KeyboardInterrupt:
        rl_fuzzer.get_logger().info("Fuzzing interrupted by user")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
