"""
semantic_control.launch.py — query + goal adapter + coordinator, 3 nodes
(layer 5 of docs/la_runbook.md §4.5). Consumes /user_command and the
landmark stream; drives Nav2 through NavigateToPose.

    ros2 launch tb3_coordinator semantic_control.launch.py
    ros2 launch tb3_coordinator semantic_control.launch.py semantic_targets_file:=/abs/path/semantic_targets.yaml

Each Node is exactly one `ros2 run` line; start them by hand when
testing one of them:

    ros2 run tb3_query       semantic_query_node.py  --ros-args --params-file <tb3_query>/config/semantic_query.yaml -p use_sim_time:=true \
                                                      -p semantic_targets_file:=<tb3_frontier_exploration>/config/semantic_targets_la.yaml
    ros2 run tb3_nav_adapter nav_goal_adapter_node   --ros-args --params-file <tb3_nav_adapter>/config/nav_goal_adapter.yaml -p use_sim_time:=true
    ros2 run tb3_coordinator coordinator_node        --ros-args --params-file <tb3_coordinator>/config/coordinator.yaml      -p use_sim_time:=true

semantic_targets_file is the registry that maps command words to detector
labels; the default is the LocateAnything one (person / bookshelf /
trash can / traffic cone / concrete barrier).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    cfg = lambda pkg, name: os.path.join(  # noqa: E731
        get_package_share_directory(pkg), "config", name)

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "semantic_targets_file",
            default_value=cfg("tb3_frontier_exploration", "semantic_targets_la.yaml"),
            description="command word -> detector label registry"),

        Node(package="tb3_query", executable="semantic_query_node.py",
             name="semantic_query_node", output="screen",
             parameters=[cfg("tb3_query", "semantic_query.yaml"),
                         {"use_sim_time": use_sim_time,
                          "semantic_targets_file": LaunchConfiguration("semantic_targets_file")}]),
        Node(package="tb3_nav_adapter", executable="nav_goal_adapter_node",
             name="nav_goal_adapter_node", output="screen",
             parameters=[cfg("tb3_nav_adapter", "nav_goal_adapter.yaml"),
                         {"use_sim_time": use_sim_time}]),
        Node(package="tb3_coordinator", executable="coordinator_node",
             name="coordinator_node", output="screen",
             parameters=[cfg("tb3_coordinator", "coordinator.yaml"),
                         {"use_sim_time": use_sim_time}]),
    ])
