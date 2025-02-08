import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, PathJoinSubstitution

def generate_launch_description():
    urdf_file = PathJoinSubstitution([
        get_package_share_directory("ds_car_description"),
        "urdf",
        "ds_car.urdf.xacro"
    ])

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": ParameterValue(
                    Command(["xacro ", urdf_file]), value_type=str
                )
            }]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.7", "0.4", "-0.2", "0", "0", "0", "base_link", "front_left_wheel"]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0.7", "-0.4", "-0.2", "0", "0", "0", "base_link", "front_right_wheel"]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["-0.7", "0.4", "-0.2", "0", "0", "0", "base_link", "rear_left_wheel"]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["-0.7", "-0.4", "-0.2", "0", "0", "0", "base_link", "rear_right_wheel"]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen"
        )
    ])
