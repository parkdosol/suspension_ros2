"""
lidar_env.py
라이더 기반 강화학습 환경 정의 파일

이 파일의 기능 :
- Gym 라이브러리를 통해 강화학습 커스텀 환경 정의함
- 강화학습 환경 개념: i번째 observation에서 에이전트가 Action을 선택하면, (i+1)번째 observation과 해당 action에 대한 reward가 반환됨

- 관측 공간(Observation Space) : (160,) vector; 1deg 간격(차량 중앙 기준 -80~80deg)의 거리 데이터 (0~30[m])
- 행동 공간(Action Space) : (4,) vector; 네 바퀴의 액티브 서스펜션 계수 (-100 ~ 100[N])
- 보상 함수(Reward Function) : 차량의 수직 가속도, 롤 각속도, 피치 각속도에 관한 함수. 승차감이 좋을수록 더 높은 보상을 받도록 설계함.

- Reset : 매 episode마다 random한 형상의 bump 생성.
- Step : 매 스텝마다 reward를 계산. 차량이 bump 기준 +5m 지점을 지나면 종료

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

        # 차량 및 bump 초기 상태
        self.vehicle_position = 0.0
        self.bump_position = 0.0

        # 환경 최초 정의 시 bump 모양 파라미터
        self.bump_height = 0.1
        self.bump_width = 3.0

    # 매 에피소드 시작 시 호출되는 함수. 환경 초기화
    def reset(self):
        if self.lidar_data is None:
            raise ValueError("Lidar data is not initialized.")

        # bump 모양 무작위 설정
        self.bump_height = np.random.uniform(0.06, 0.1)  # height
        self.bump_width = np.random.uniform(2.4, 3.6)  # width

        # bump 위치는 고정 (예: x = 0)
        self.bump_position = 0.0

        # 차량은 bump 기준 -50m에서 시작
        self.vehicle_position = self.bump_position - 50.0

        return self.lidar_data  # reset 함수의 return은 1st observation 이어야 함

    # Reward 함수 정의
    def compute_reward(self, action):
        vertical_penalty = abs(self.vertical_acceleration)
        roll_penalty = abs(self.roll_angular_velocity)
        pitch_penalty = abs(self.pitch_angular_velocity)

        total_penalty = 1.0 * vertical_penalty + 1.0 * roll_penalty + 1.0 * pitch_penalty  # weight는 추후 변경 예정

        reward = -total_penalty

        return reward

    # 매 스텝마다 호출되는 함수. 환경 업데이트 및 Reward 계산
    def step(self, action):
        reward = self.compute_reward(action)

        # 종료 조건 설정: 차량이 bump 기준 +5m 지점을 지나면 종료
        done = self.vehicle_position >= (self.bump_position + 5.0)

        return self.lidar_data, reward, done, {}

    # ROS2로부터 데이터를 업데이트하는 함수들 (ROS2 노드에서 호출)

    def update_lidar_data(self, value):
        self.lidar_data = value

    def update_vertical_acceleration(self, value):
        self.vertical_acceleration = value

    def update_roll_angular_velocity(self, value):
        self.roll_angular_velocity = value

    def update_pitch_angular_velocity(self, value):
        self.pitch_angular_velocity = value

    def update_vehicle_position(self, value):
        self.vehicle_position = value


