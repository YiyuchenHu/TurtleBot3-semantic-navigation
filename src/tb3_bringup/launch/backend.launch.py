"""
backend.launch.py — Terminal 3: the provided course backend.

Starts every provided node between perception and Nav2 in one shot:

  semantic pipeline   semantic_memory_node, semantic_map_memory_node,
                      semantic_query_node, nav_goal_adapter_node,
                      coordinator_node
  exploration         startup_map_warmup_node, frontier_detection_node,
                      goal_assignment_node

NOT started here (each has its own terminal / launch):
  simulation (sim.launch.py), SLAM+Nav2+RViz (nav.launch.py),
  detector (tb3_detector), localizer (tb3_localizer).

Start this AFTER nav.launch.py reports Nav2 active: the warmup node
drives a scan rotation via /cmd_vel and the coordinator/goal-assignment
nodes look for the navigate_to_pose action. Started earlier (or alone)
the nodes just wait and retry — they do not crash.

Usage:
    ros2 launch tb3_bringup backend.launch.py
    ros2 launch tb3_bringup backend.launch.py use_runtime_debug:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    pkg_fe = get_package_share_directory("tb3_frontier_exploration")
    fe_config = os.path.join(pkg_fe, "config", "params.yaml")

    map_topic = LaunchConfiguration("map_topic", default="/map")
    costmap_topic = LaunchConfiguration(
        "costmap_topic", default="/global_costmap/costmap")
    odom_topic = LaunchConfiguration("odom_topic", default="/odometry/filtered")

    return LaunchDescription([
        # This backend only makes sense next to the Gazebo sim, so sim
        # time defaults to true here (unlike the per-package launches).
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_runtime_debug", default_value="false",
                              description="Also start semantic_runtime_debug_node"),
        DeclareLaunchArgument("map_topic", default_value="/map",
                              description="OccupancyGrid topic for the map"),
        DeclareLaunchArgument("costmap_topic", default_value="/global_costmap/costmap",
                              description="Costmap topic for frontier cost sampling"),
        DeclareLaunchArgument("odom_topic", default_value="/odometry/filtered",
                              description="Odometry topic for goal assignment"),

        # ── Semantic pipeline (between localizer and Nav2) ───────────────
        Node(
            package="tb3_memory",
            executable="semantic_memory_node",
            name="semantic_memory_node",
            output="screen",
            parameters=[
                PathJoinSubstitution([FindPackageShare("tb3_memory"),
                                      "config", "semantic_memory.yaml"]),
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="tb3_coordinator",
            executable="semantic_map_memory_node",
            name="semantic_map_memory_node",
            output="screen",
            parameters=[
                PathJoinSubstitution([FindPackageShare("tb3_coordinator"),
                                      "config", "coordinator.yaml"]),
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="tb3_query",
            executable="semantic_query_node.py",
            name="semantic_query_node",
            output="screen",
            parameters=[
                PathJoinSubstitution([FindPackageShare("tb3_query"),
                                      "config", "semantic_query.yaml"]),
                # semantic_targets.yaml is an asset, not tb3_query's own
                # config: it lives in tb3_bringup/config next to the worlds
                # whose objects it names. Set explicitly here so the path is
                # visible in the launch file rather than resolved by a
                # cross-package fallback inside the node.
                {"semantic_targets_file": PathJoinSubstitution(
                    [FindPackageShare("tb3_bringup"),
                     "config", "semantic_targets.yaml"])},
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="tb3_nav_adapter",
            executable="nav_goal_adapter_node",
            name="nav_goal_adapter_node",
            output="screen",
            parameters=[
                PathJoinSubstitution([FindPackageShare("tb3_nav_adapter"),
                                      "config", "nav_goal_adapter.yaml"]),
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="tb3_coordinator",
            executable="coordinator_node",
            name="coordinator_node",
            output="screen",
            parameters=[
                PathJoinSubstitution([FindPackageShare("tb3_coordinator"),
                                      "config", "coordinator.yaml"]),
                {"use_sim_time": use_sim_time},
            ],
        ),

        # ── Frontier exploration ─────────────────────────────────────────
        # One-shot cmd_vel rotation scan; signals exploration_warmup_complete
        # when done.
        Node(
            package="tb3_frontier_exploration",
            executable="startup_map_warmup_node.py",
            name="startup_map_warmup_node",
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="tb3_frontier_exploration",
            executable="frontier_detection_node",
            name="frontier_detection_node",
            parameters=[
                fe_config,
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
                fe_config,
                {"use_sim_time": use_sim_time},
                # Same flattening bug as above. Note the subscription this feeds
                # is an intentional no-op hook (goal_assignment_node.cpp
                # odomCallback); the argument is kept working so the topic name
                # stays overridable, not because pose comes from it.
                {"odom_topic": odom_topic},
            ],
            output="screen",
        ),

        # ── Runtime debug diagnostics (opt-in) ───────────────────────────
        Node(
            package="tb3_coordinator",
            executable="semantic_runtime_debug_node",
            name="semantic_runtime_debug_node",
            parameters=[{"use_sim_time": use_sim_time}],
            condition=IfCondition(LaunchConfiguration("use_runtime_debug")),
            output="screen",
        ),
    ])
