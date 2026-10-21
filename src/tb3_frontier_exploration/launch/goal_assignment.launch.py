import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("tb3_frontier_exploration")
    config = os.path.join(pkg_share, "config", "params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    odom_topic = LaunchConfiguration("odom_topic", default="/odometry/filtered")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true",
                             description="Use simulation time"),
        DeclareLaunchArgument("odom_topic", default_value="/odometry/filtered",
                             description="Odometry topic for robot pose (e.g. from robot_localization)"),

        Node(
            package="tb3_frontier_exploration",
            executable="startup_map_warmup_node.py",
            name="startup_map_warmup_node",
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="tb3_frontier_exploration",
            executable="goal_assignment_node",
            name="goal_assignment_node",
            parameters=[
                config,
                {"use_sim_time": use_sim_time},
                # Flat {name: value} — a nested
                # {"goal_assignment_node": {"ros__parameters": {...}}} dict is
                # flattened by launch_ros into a literal dotted parameter name
                # and the odom_topic launch argument silently did nothing. Note
                # the subscription this feeds is an intentional no-op hook
                # (goal_assignment_node.cpp odomCallback); the argument is kept
                # working so the topic name stays overridable, not because pose
                # comes from it.
                {"odom_topic": odom_topic},
            ],
            output="screen",
        ),
    ])
