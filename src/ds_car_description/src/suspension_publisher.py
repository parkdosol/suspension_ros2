#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_msgs.msg import Suspension  # custom_msgs 패키지에서 인터페이스 가져오기
import random

class SuspensionPublisher(Node):
    def __init__(self):
        super().__init__('suspension_publisher')
        # 'suspension_states' 토픽으로 메시지 퍼블리시, 큐 사이즈 10
        self.publisher_ = self.create_publisher(Suspension, 'suspension_states', 10)
        # 0.1초마다 타이머 콜백 호출 (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
    
    def timer_callback(self):
        msg = Suspension()
        # 임의의 값으로 각 서스펜션 변위를 시뮬레이션 (예: -0.05 ~ 0.05 미터)
        msg.left_front  = random.uniform(-0.05, 0.05)
        msg.right_front = random.uniform(-0.05, 0.05)
        msg.left_rear   = random.uniform(-0.05, 0.05)
        msg.right_rear  = random.uniform(-0.05, 0.05)
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Publishing: LF={msg.left_front:.3f}, RF={msg.right_front:.3f}, '
            f'LR={msg.left_rear:.3f}, RR={msg.right_rear:.3f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = SuspensionPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
