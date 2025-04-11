"""
run_agent.py
서스펜션 제어 강화를 위한 에이전트 실행 스크립트

이 파일의 기능 :
- 강화학습 에이전트를 초기화하고, ROS2 노드를 실행하며,
  학습(train) 또는 평가(test)를 수행합니다.

Created by: Youngju Park
Date: April 5, 2025
"""

import rclpy
from agent import SuspensionAgent

def main(args=None):
    rclpy.init(args=args)
    agent_node = SuspensionAgent()

    try:
        # ===== 사용 모드 선택 =====
        MODE = "train"  # 또는 "test"

        if MODE == "train":
            agent_node.train(episodes=100)  # 학습
        elif MODE == "test":
            agent_node.test(episodes=5)     # 테스트
        else:
            agent_node.get_logger().error(f"Unknown mode: {MODE}")

    except KeyboardInterrupt:
        pass

    finally:
        agent_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

