"""
localizer.launch.py — Terminal 4: the Stage-2 planar localizer.

Usage:
    ros2 launch tb3_bringup localizer.launch.py                       # sim (default)
    ros2 launch tb3_bringup localizer.launch.py use_sim_time:=false   # real robot
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("tb3_localizer")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description=(
                "Course default: Gazebo simulation time. "
                "Pass false when reusing this node on a real robot."
            ),
        ),

        Node(
            package="tb3_localizer",
            executable="localizer_node",
            name="localizer_node",
            output="screen",
            parameters=[
                PathJoinSubstitution([pkg_share, "config", "localizer.yaml"]),
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
            ],
        ),
    ])
