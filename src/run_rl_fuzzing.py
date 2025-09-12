#!/usr/bin/env python3
"""
강화학습 기반 RoboFuzz 실행 스크립트
기존 fuzzer.py의 명령행 인터페이스를 확장하여 RL 기능 추가
"""

import argparse
import sys
import os
import rclpy
from rl_fuzzer_integration import RLFuzzerIntegration

def main():
    parser = argparse.ArgumentParser(description='RL-Enhanced RoboFuzz')
    
    # 기존 RoboFuzz 인수들
    parser.add_argument('--px4-sitl-ros', action='store_true', 
                       help='Enable PX4 SITL with ROS2')
    parser.add_argument('--px4-mission', type=str, 
                       help='PX4 mission file path')
    parser.add_argument('--seed', type=int, default=42,
                       help='Random seed')
    parser.add_argument('--cycles', type=int, default=100,
                       help='Number of fuzzing cycles')
    
    # RL 관련 인수들
    parser.add_argument('--rl-enabled', action='store_true', default=True,
                       help='Enable RL-based mission generation')
    parser.add_argument('--rl-ratio', type=float, default=0.3,
                       help='Ratio of RL missions (0.0-1.0)')
    parser.add_argument('--rl-learning-rate', type=float, default=0.001,
                       help='RL learning rate')
    parser.add_argument('--rl-epsilon', type=float, default=1.0,
                       help='RL exploration rate')
    parser.add_argument('--rl-epsilon-decay', type=float, default=0.995,
                       help='RL epsilon decay rate')
    parser.add_argument('--rl-memory-size', type=int, default=10000,
                       help='RL replay buffer size')
    parser.add_argument('--rl-batch-size', type=int, default=32,
                       help='RL training batch size')
    
    # 미션 생성 관련
    parser.add_argument('--mission-type', type=str, default='mixed',
                       choices=['rl', 'static', 'mixed'],
                       help='Mission generation type')
    parser.add_argument('--min-waypoints', type=int, default=20,
                       help='Minimum number of waypoints')
    parser.add_argument('--max-waypoints', type=int, default=100,
                       help='Maximum number of waypoints')
    parser.add_argument('--target-bugs', nargs='+', 
                       default=['acceleration_overflow', 'gps_inconsistency'],
                       help='Target bug types')
    
    # 출력 및 로깅
    parser.add_argument('--output-dir', type=str, default='/robofuzz/src/missions',
                       help='Output directory for generated missions')
    parser.add_argument('--log-level', type=str, default='INFO',
                       choices=['DEBUG', 'INFO', 'WARN', 'ERROR'],
                       help='Logging level')
    parser.add_argument('--save-model', type=str, default=None,
                       help='Path to save trained RL model')
    parser.add_argument('--load-model', type=str, default=None,
                       help='Path to load pre-trained RL model')
    
    args = parser.parse_args()
    
    # 설정 객체 생성
    class Config:
        def __init__(self, args):
            # 기본 RoboFuzz 설정
            self.px4_sitl = args.px4_sitl_ros
            self.px4_ros = args.px4_sitl_ros
            self.use_mavlink = False
            self.exp_pgfuzz = False
            self.rospkg = "px4"
            self.rosnode = "px4"
            self.exec_cmd = "px4"
            self.seed = args.seed
            self.px4_mission_file = args.px4_mission
            
            # RL 설정
            self.rl_enabled = args.rl_enabled
            self.rl_learning_rate = args.rl_learning_rate
            self.rl_mission_ratio = args.rl_ratio
            self.rl_epsilon = args.rl_epsilon
            self.rl_epsilon_decay = args.rl_epsilon_decay
            self.rl_memory_size = args.rl_memory_size
            self.rl_batch_size = args.rl_batch_size
            
            # 미션 생성 설정
            self.mission_type = args.mission_type
            self.min_waypoints = args.min_waypoints
            self.max_waypoints = args.max_waypoints
            self.target_bugs = args.target_bugs
            
            # 출력 설정
            self.output_dir = args.output_dir
            self.log_level = args.log_level
            self.save_model = args.save_model
            self.load_model = args.load_model
    
    config = Config(args)
    
    # 출력 디렉토리 생성
    os.makedirs(config.output_dir, exist_ok=True)
    
    # RCL 초기화
    rclpy.init()
    
    try:
        # RL Fuzzer 통합 실행
        rl_fuzzer = RLFuzzerIntegration(config)
        
        # 사전 훈련된 모델 로드 (있는 경우)
        if config.load_model and os.path.exists(config.load_model):
            rl_fuzzer.get_logger().info(f"Loading pre-trained model from {config.load_model}")
            # 모델 로드 로직 구현 필요
        
        # 적응형 퍼징 실행
        rl_fuzzer.get_logger().info(f"Starting RL-enhanced fuzzing for {args.cycles} cycles")
        rl_fuzzer.run_adaptive_fuzzing(num_cycles=args.cycles)
        
        # 모델 저장 (요청된 경우)
        if config.save_model:
            rl_fuzzer.get_logger().info(f"Saving trained model to {config.save_model}")
            # 모델 저장 로직 구현 필요
        
    except KeyboardInterrupt:
        print("\nFuzzing interrupted by user")
    except Exception as e:
        print(f"Error during fuzzing: {e}")
        sys.exit(1)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
