"""
lidar_env.py

This file defines the environment used for reinforcement learning.
It processes Lidar data received from ROS2 and evaluates the performance
of suspension control for a vehicle.

The environment provides:
- Observation space: Lidar data array of 360 points representing terrain characteristics.
- Action space: Suspension coefficients for four wheels [k1, k2, k3, k4].
- Reward function: Based on vertical acceleration, roll angular velocity, and pitch angular velocity.

Created by: Youngju Park
Date: April 5, 2025
"""

import numpy as np
from gym import spaces, Env


class SuspensionEnv(Env):
    def __init__(self, lidar_data=None):
        super(SuspensionEnv, self).__init__()
        self.observation_space = spaces.Box(low=0, high=30, shape=(160,),
                                            dtype=np.float32)  # Obs : (160,) vector; 1deg 간격(차량 중앙 기준 -80~80deg)의 거리 데이터 (0~30[m])
        self.action_space = spaces.Box(low=-100, high=100, shape=(4,),
                                       dtype=np.float32)  # Action : (4,) vector; 네 바퀴의 액티브 서스펜션 계수 (-100 ~ 100[N])

        self.lidar_data = lidar_data

        # ROS2로부터 수신되는 데이터 초기화
        self.vertical_acceleration = 0.0
        self.roll_angular_velocity = 0.0
        self.pitch_angular_velocity = 0.0

    # 매 에피소드 시작 시 호출되는 함수. 환경 초기화
    def reset(self):
        if self.lidar_data is None:
            raise ValueError("Lidar data is not initialized. Make sure ROS2 is providing data correctly.")

        # ROS2로부터 데이터를 계속 받아야 하므로, 초기화 작업 없이 현재 lidar_data 그대로 반환
        return self.lidar_data

    # Reward 함수 정의
    def compute_reward(self, action):
        # 차량 상태 정보로부터 페널티 계산 (ROS2로부터 받은 값 사용)
        vertical_penalty = abs(self.vertical_acceleration)
        roll_penalty = abs(self.roll_angular_velocity)
        pitch_penalty = abs(self.pitch_angular_velocity)

        # 전체 페널티 계산 (가중치 조정 필요!)
        total_penalty = 1.0 * vertical_penalty + 1.0 * roll_penalty + 1.0 * pitch_penalty

        # 보상 계산 (값이 작을수록 더 좋은 보상)
        reward = - total_penalty  # 페널티 값을 음수로 하여 최소화할수록 보상이 증가하도록 설정

        return reward


    # 매 스텝마다 호출되는 함수. 환경 업데이트 및 Reward 계산
    def step(self, action):
        reward = self.compute_reward(action)

        # 종료 조건 설정 *** 종료 조건 지정 필요(How?)
        done = False

        return self.lidar_data, reward, done, {}


    # ROS2로부터 데이터를 업데이트하는 함수들 (ROS2 노드에서 호출)
    # 후에 데이터 처리 과정에서 검사를 용이하게 하기 위해 업데이트 함수를 통해 값을 저장함

    # Lidar 데이터 업데이트 함수
    def update_lidar_data(self, value):
        self.lidar_data = value


    def update_vertical_acceleration(self, value):
        self.vertical_acceleration = value


    def update_roll_angular_velocity(self, value):
        self.roll_angular_velocity = value


    def update_pitch_angular_velocity(self, value):
        self.pitch_angular_velocity = value

