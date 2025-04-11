"""
agent.py
서스펜션 제어를 위한 강화학습 에이전트 정의 파일

이 파일의 기능 :
- 차량의 승차감을 향상시키기 위해 서스펜션 계수를 학습하는 강화학습 에이전트를 구현
- ROS2와 통신하여 Lidar 데이터를 수신하고, 네 바퀴 서스펜션 계수를 퍼블리시
- 차량의 상태 데이터(수직 가속도, 롤 각속도, 피치 각속도) 수신

Created by: Youngju Park
Date: April 5, 2025
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from sensor_msgs.msg import LaserScan
from stable_baselines3 import PPO
from lidar_env import SuspensionEnv
import numpy as np
from custom_msgs.msg import BumpInfo, VehicleInit


class SuspensionAgent(Node):
    def __init__(self):
        super().__init__('suspension_agent')  # suspension_agent 라는 이름의 Node를 생성

        #
        self.env = SuspensionEnv()  # 환경 객체 생성
        self.model = PPO("MlpPolicy", self.env, verbose=1)  # 모델 객체 생성

        # ========================== 통신 파이프라인 설정  ==========================

        # Publisher Part
        # self.<퍼블리셔 변수명> = self.create_publisher(msg type, topic name, que size)
        # (예시) 이걸 정의해놓으면 나중에 self.publisher.publish(msg) 함수를 통해 이 토픽에 데이터를 퍼블리시 할 수 있음

        # 서스펜션 계수 퍼블리셔
        self.publisher = self.create_publisher(Float32MultiArray, '/suspension_coefficients', 10)

        # bump 정보 퍼블리셔
        self.bump_pub = self.create_publisher(BumpInfo, '/bump_info', 10)

        # 차량 초기 위치 퍼블리셔
        self.vehicle_pub = self.create_publisher(VehicleInit, '/vehicle_init', 10)

        # Subscriber Part
        # self.<서브스크라이버 변수명> = self.create_subscription(msg type, topic name, callback function, que size)
        # (예시) 이걸 정의해놓으면 나중에 ROS2에서 /scan 토픽으로 LaserScan 타입의 메시지가 발행되면 self.lidar_callback 함수가 자동으로 호출됨

        # Lidar 데이터 구독
        self.subscription_lidar = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # 차량 상태 데이터 구독
        self.subscription_acceleration = self.create_subscription(Float32, '/vertical_acceleration',
                                                                  self.acceleration_callback, 10)

        self.subscription_roll = self.create_subscription(Float32, '/roll_angular_velocity', self.roll_callback, 10)

        self.subscription_pitch = self.create_subscription(Float32, '/pitch_angular_velocity', self.pitch_callback, 10)

    # ========================== 데이터 송수신 함수 정의  ==========================

    # Lidar 데이터 수신 콜백 함수
    def lidar_callback(self, msg):
        lidar_data = np.array(msg.ranges[:160])  # 데이터를 np array로 변환
        self.env.update_lidar_data(lidar_data)  # 데이터를 env에 넘겨줌

    # 차량 상태 데이터 콜백 함수
    def acceleration_callback(self, msg):
        self.env.update_vertical_acceleration(msg.data)  # 데이터를 env에 넘겨줌

    def roll_callback(self, msg):
        self.env.update_roll_angular_velocity(msg.data)

    def pitch_callback(self, msg):
        self.env.update_pitch_angular_velocity(msg.data)

    # 서스펜션 계수 퍼블리시하는 함수
    def publish_action(self, action):
        msg = Float32MultiArray()  # msg 객체 생성
        msg.data = action.tolist()  # numpy array → list로 변환
        self.publisher.publish(msg)  # publisher 파이프라인(topic : /suspension_coefficients)으로 msg 발행
        self.get_logger().info(f'Published suspension coefficients: {action}')  # 로그 출력

    # bump 정보 퍼블리시 (height, width)
    def publish_bump_info(self):
        msg = BumpInfo()
        msg.height = self.env.bump_height
        msg.width = self.env.bump_width
        self.bump_pub.publish(msg)
        self.get_logger().info(f'Published bump info: height={msg.height}, width={msg.width}')

    # 차량 초기 위치 퍼블리시 (x좌표)
    def publish_vehicle_position(self):
        msg = VehicleInit()
        msg.x = self.env.vehicle_position
        self.vehicle_pub.publish(msg)
        self.get_logger().info(f'Published vehicle init position: x={msg.x}')

        # ========================== Training 함수 정의  ==========================

    def train(self, episodes, save_path="suspension_agent_model"):
        """
        학습 루프:
        - 각 에피소드마다 bump 및 차량 위치 초기화
        - Lidar 데이터 기반으로 관측값 받고, action 선택
        - 선택된 action을 ROS2에 퍼블리시하고 reward 계산
        - 차량이 bump 기준 +5m 지점을 넘으면 에피소드 종료
        """

        for ep in range(episodes):  # 매 episode에 대하여...
            obs = self.env.reset()  # env reset
            self.publish_bump_info()  # bump info를 ROS2에 publish
            self.publish_vehicle_position()  # vehicle position을

            total_reward = 0.0
            step_count = 0

            done = False
            while not done:  # 매 step에 대하여...
                action, _ = self.model.predict(obs)  # agent 함수에게 obs 넣고 action 받음
                self.publish_action(
                    action)  # action을 ROS2에 publish -> ROS2 환경에 변화가 일어남 -> ROS2로부터 새로운 Lidar 및 차량 상태 데이터를 받음.
                obs, reward, done, _ = self.env.step(action)  # 새롭게 변한 obs, 차량 상태 데이터로부터 얻어진 reward, done flag를 계산
                total_reward += reward
                step_count += 1

            self.get_logger().info(
                f"[Episode {ep + 1}] finished: Total steps = {step_count}, Total reward = {total_reward:.2f}"
            )

        # Training 종료 후...
        self.model.save(save_path)
        self.get_logger().info(f"모델 저장 완료: '{save_path}'")

    # ========================== Test 함수 정의  ==========================
    def test(self, episodes, load_path="suspension_agent_model"):
        """
        테스트 루프:
        - 학습된 모델을 사용하여 에이전트를 평가
        - 학습 없이 predict만 사용
        - 각 에피소드에 대한 총 보상 출력
        """

        self.load(load_path)  # 저장된 모델 로드

        for ep in range(episodes):
            obs = self.env.reset()
            self.publish_bump_info()
            self.publish_vehicle_position()

            total_reward = 0.0
            step_count = 0

            done = False
            while not done:
                action, _ = self.model.predict(obs, deterministic=True)  # 테스트에서는 deterministic=True
                self.publish_action(action)
                obs, reward, done, _ = self.env.step(action)
                total_reward += reward
                step_count += 1

            self.get_logger().info(
                f"[Test Episode {ep + 1}] 종료: 총 스텝 수 = {step_count}, 총 보상 = {total_reward:.2f}"
            )

    # ========================== 데이터 로드 함수 정의  ==========================
    def load(self, path="suspension_agent_model"):
        self.model.load(path)
