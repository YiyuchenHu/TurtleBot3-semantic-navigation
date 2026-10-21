"""
perception_la.launch.py — the LocateAnything perception chain, 4 nodes
(layer 3 of the per-layer manual bring-up). Nothing else: no Gazebo, no Nav2,
no exploration, no coordinator.

    ros2 launch tb3_coordinator perception_la.launch.py
    ros2 launch tb3_coordinator perception_la.launch.py detector_config:=/abs/path/la_detector_search_first.yaml

Each Node below is exactly one `ros2 run` line (same executable, same
--params-file); start them one at a time by hand when testing a single
node — the launch file adds nothing the nodes depend on.

    ros2 run tb3_detector    la_detector_node          --ros-args --params-file <tb3_detector>/config/la_detector.yaml       -p use_sim_time:=true
    ros2 run tb3_localizer   localizer_node            --ros-args --params-file <tb3_localizer>/config/localizer.yaml        -p use_sim_time:=true
    ros2 run tb3_memory      semantic_memory_node      --ros-args --params-file <tb3_memory>/config/semantic_memory.yaml     -p use_sim_time:=true
    ros2 run tb3_coordinator semantic_map_memory_node  --ros-args --params-file <tb3_coordinator>/config/coordinator.yaml    -p use_sim_time:=true

Data flow: /camera/image_raw -> la_detector_node (HTTP /detect on the
grounding container) -> /detector_node/detections -> localizer_node
(+ /scan) -> /localizer_node/localized_objects -> semantic_memory_node
-> /semantic_memory_node/objects -> semantic_map_memory_node (+ /map, TF)
-> /semantic_map_memory_node/landmark_objects + /semantic_memory_markers.
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
            "detector_config", default_value=cfg("tb3_detector", "la_detector.yaml"),
            description="la_detector_node parameter file (vocabulary profile, "
                        "docs/la_runbook.md §7)"),

        Node(package="tb3_detector", executable="la_detector_node",
             name="la_detector_node", output="screen",
             parameters=[LaunchConfiguration("detector_config"),
                         {"use_sim_time": use_sim_time}]),
        Node(package="tb3_localizer", executable="localizer_node",
             name="localizer_node", output="screen",
             parameters=[cfg("tb3_localizer", "localizer.yaml"),
                         {"use_sim_time": use_sim_time}]),
        Node(package="tb3_memory", executable="semantic_memory_node",
             name="semantic_memory_node", output="screen",
             parameters=[cfg("tb3_memory", "semantic_memory.yaml"),
                         {"use_sim_time": use_sim_time}]),
        Node(package="tb3_coordinator", executable="semantic_map_memory_node",
             name="semantic_map_memory_node", output="screen",
             parameters=[cfg("tb3_coordinator", "coordinator.yaml"),
                         {"use_sim_time": use_sim_time}]),
    ])
