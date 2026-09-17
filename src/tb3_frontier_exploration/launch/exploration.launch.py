"""
exploration.launch.py — frontier exploration, 3 nodes (layer 4 of
docs/la_runbook.md §4.4).

    ros2 launch tb3_frontier_exploration exploration.launch.py
    ros2 launch tb3_frontier_exploration exploration.launch.py warmup_angular_speed:=0.3
    ros2 launch tb3_frontier_exploration exploration.launch.py start_enabled:=true   # no coordinator: explore from boot

Each Node is exactly one `ros2 run` line; start them by hand when testing
one of them (params.yaml already carries map_topic / costmap_topic /
odom_topic, the launch arguments below only override it):

    ros2 run tb3_frontier_exploration startup_map_warmup_node.py --ros-args -p use_sim_time:=true -p scan_angular_speed:=0.4
    ros2 run tb3_frontier_exploration frontier_detection_node    --ros-args --params-file <pkg>/config/params.yaml -p use_sim_time:=true
    ros2 run tb3_frontier_exploration goal_assignment_node       --ros-args --params-file <pkg>/config/params.yaml -p use_sim_time:=true
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("tb3_frontier_exploration")
    config = os.path.join(pkg_share, "config", "params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    map_topic = LaunchConfiguration("map_topic", default="/map")
    costmap_topic = LaunchConfiguration("costmap_topic", default="/global_costmap/costmap")
    odom_topic = LaunchConfiguration("odom_topic", default="/odometry/filtered")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true",
                             description="Use simulation time"),
        DeclareLaunchArgument("map_topic", default_value="/map",
                             description="OccupancyGrid topic for the map"),
        DeclareLaunchArgument("costmap_topic", default_value="/global_costmap/costmap",
                             description="Costmap topic for frontier cost sampling"),
        DeclareLaunchArgument("odom_topic", default_value="/odometry/filtered",
                             description="Odometry topic for goal assignment"),

        DeclareLaunchArgument("start_enabled", default_value="false",
                              description="explore before /exploration_enabled arrives "
                                          "(standalone use without a coordinator)"),
        DeclareLaunchArgument("warmup_angular_speed", default_value="0.4",
                             description="Start-up rotation scan speed (rad/s)"),

        # One-shot cmd_vel rotation scan; signals exploration_warmup_complete when done.
        Node(
            package="tb3_frontier_exploration",
            executable="startup_map_warmup_node.py",
            name="startup_map_warmup_node",
            # ParameterValue(float): a bare LaunchConfiguration is
            # yaml-parsed, so `warmup_angular_speed:=1` would arrive as an
            # INTEGER and be rejected by the node's DOUBLE declaration.
            parameters=[{"use_sim_time": use_sim_time,
                         "scan_angular_speed": ParameterValue(
                             LaunchConfiguration("warmup_angular_speed"),
                             value_type=float)}],
            output="screen",
        ),
        Node(
            package="tb3_frontier_exploration",
            executable="frontier_detection_node",
            name="frontier_detection_node",
            parameters=[
                config,
                {"use_sim_time": use_sim_time},
                # Flat {name: value}. A dict entry in `parameters` is a literal
                # override list, not a YAML document: launch_ros flattens any
                # nesting into dotted names, so the old
                # {"frontier_detection_node": {"ros__parameters": {...}}} form
                # produced undeclared parameters named
                # "frontier_detection_node.ros__parameters.map_topic" and the
                # two launch arguments silently did nothing. The node-name /
                # ros__parameters wrapper belongs only in the YAML file above.
                {"map_topic": map_topic,
                 "costmap_topic": costmap_topic},
            ],
            output="screen",
        ),
        Node(
            package="tb3_frontier_exploration",
            executable="goal_assignment_node",
            name="goal_assignment_node",
            parameters=[
                config,
                {"use_sim_time": use_sim_time},
                # Same flattening bug as above. Note the subscription this feeds
                # is an intentional no-op hook (goal_assignment_node.cpp
                # odomCallback); the argument is kept working so the topic name
                # stays overridable, not because pose comes from it.
                {"odom_topic": odom_topic,
                 "start_enabled": ParameterValue(
                     LaunchConfiguration("start_enabled"), value_type=bool)},
            ],
            output="screen",
        ),
    ])
