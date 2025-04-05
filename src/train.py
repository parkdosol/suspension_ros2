"""
train.py

This file serves as the training script for the reinforcement learning agent.
It initializes the agent, starts the ROS2 node, and trains the model using the specified environment.

Usage:
    python train.py

Created by: Youngju Park
Date: April 5, 2025
"""

import rclpy
from agent import SuspensionAgent


def main(args=None):
    rclpy.init(args=args)
    agent_node = SuspensionAgent()

    try:
        rclpy.spin(agent_node)
    except KeyboardInterrupt:
        pass

    agent_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

