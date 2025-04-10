"""
agent.py

This file implements the reinforcement learning agent responsible for learning
optimal suspension coefficients to enhance ride comfort. It communicates with ROS2
to receive Lidar data and publish suspension coefficients, as well as receiving
vehicle state data (vertical acceleration, roll angular velocity, pitch angular velocity).

Dependencies:
- stable-baselines3
- torch
- rclpy (ROS2 Python API)

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


class SuspensionAgent(Node):
    def __init__(self):
        super().__init__('suspension_agent')

        self.env = SuspensionEnv()  # 환경 객체 생성
        self.model = PPO("MlpPolicy", self.env, verbose=1)  # 모델 객체 생성

        # ========================== 통신 파이프라인 설정  ==========================
        # publish 초기화
        self.publisher = self.create_publisher(Float32MultiArray,  # 메시지 타입 : ROS2로 퍼블리시할 서스펜션 계수 (Float32)
                                               '/suspension_coefficients',  # 퍼블리시할 토픽 이름
                                               10  # 큐 사이즈: 메시지를 버퍼링할 수 있는 최대 수
                                               )

        # subscribe 초기화
        # (1) Lidar 데이터
        self.subscription_lidar = self.create_subscription(  # <-- 변수명 변경
            LaserScan,  # 메시지 타입 : ROS2 시뮬레이션에서 보내는 Lidar 데이터 (LaserScan)
            '/scan',  # 토픽 이름: ROS2 시뮬레이션에서 퍼블리시되는 Lidar 데이터 토픽
            self.lidar_callback,  # 콜백 함수: 메시지를 수신할 때 호출되는 함수 (아래에 정의되어 있음)
            10  # 큐 사이즈: 메시지를 버퍼링할 수 있는 최대 수
        )

        # (2) 차량 상태 데이터
        self.subscription_acceleration = self.create_subscription(
            Float32,
            '/vertical_acceleration',
            self.acceleration_callback,
            10
        )

        self.subscription_roll = self.create_subscription(
            Float32,
            '/roll_angular_velocity',
            self.roll_callback,
            10
        )

        self.subscription_pitch = self.create_subscription(
            Float32,
            '/pitch_angular_velocity',
            self.pitch_callback,
            10
        )

        self.lidar_data = None

    # ========================== 데이터 송수신 함수 정의  ==========================

    # Subscriber, 데이터 수신 (ROS2 -> Agent)
    # 이 콜백 함수들은 ROS2 퍼블리셔가 메시지를 보낼 때(ROS2 -> Agent) ROS2 시스템에서 자동으로 호출됨

    # Lidar 데이터 수신 콜백 함수
    def lidar_callback(self, msg):
        lidar_data = np.array(msg.ranges[:160])  # Lidar 데이터를 160개의 값으로 자르고 numpy 배열로 변환
        self.env.update_lidar_data(lidar_data)  # 환경 객체로 데이터 전달


    # Vertical acceleration 수신 콜백 함수 (ROS2 -> Agent)
    def acceleration_callback(self, msg):
        self.env.update_vertical_acceleration(msg.data)  # 환경 객체로 데이터 전달


    # Roll angular velocity 수신 콜백 함수 (ROS2 -> Agent)
    def roll_callback(self, msg):
        self.env.update_roll_angular_velocity(msg.data)  # 환경 객체로 데이터 전달


    # Pitch angular velocity 수신 콜백 함수 (ROS2 -> Agent)
    def pitch_callback(self, msg):
        self.env.update_pitch_angular_velocity(msg.data)  # 환경 객체로 데이터 전달


    # Publisher, 데이터 송신 (Agent -> ROS2 시뮬레이션 환경)
    def publish_action(self, action):
        msg = Float32MultiArray()
        msg.data = action.tolist()  # 액션(numpy array)을 리스트로 변환하여 메시지에 저장
        self.publisher.publish(msg)  # ROS2 토픽으로 퍼블리시
        self.get_logger().info(f'Published suspension coefficients: {action}')


    # ========================== Training 함수 정의  ==========================
    def train(self, timesteps=10000):
        self.model.learn(total_timesteps=timesteps)
        self.model.save("suspension_agent_model")


    # ========================== 데이터 로드 함수 정의  ==========================
    def load(self, path="suspension_agent_model"):
        self.model.load(path)
