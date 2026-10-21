"""
detector.launch.py — Terminal 5: the YOLO detector.

Usage:
    ros2 launch tb3_bringup detector.launch.py
    ros2 launch tb3_bringup detector.launch.py device:=cuda:0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("tb3_detector")

    # ── Launch arguments ────────────────────────────────────────────────
    # Only three parameters are controlled from launch:
    #
    #   model_path   — resolved to an absolute install path here so the node
    #                  does not have to guess where pkg_share lives at runtime.
    #   device       — useful to flip to "cuda:0" without editing the yaml.
    #   use_sim_time — must be set at launch time (clock source is external).
    #
    # ALL other parameters (conf_threshold, class_filter, publish_debug_image,
    # image_topic, camera_info_topic, enable_tracking) come exclusively from
    # detector.yaml.  Edit that file and restart the node to change them.
    return LaunchDescription([
        DeclareLaunchArgument(
            "model_path",
            default_value=PathJoinSubstitution([pkg_share, "models", "tb3det_yolo26n.pt"]),
            description="Absolute path to YOLO26 .pt weights.",
        ),
        DeclareLaunchArgument(
            "device",
            default_value="cpu",
            description="Torch inference device: 'cpu' or 'cuda:0'.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description=(
                "Course default: Gazebo simulation time. "
                "Pass false when reusing this node on a real robot."
            ),
        ),

        # ── detector_node ────────────────────────────────────────────────
        # Parameter resolution order (later entries win):
        #   1. detector.yaml  — provides all defaults
        #   2. inline dict    — overrides only the three launch-controlled keys
        Node(
            package="tb3_detector",
            executable="detector_node",
            name="detector_node",
            output="screen",
            parameters=[
                PathJoinSubstitution([pkg_share, "config", "detector.yaml"]),
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "model_path":   LaunchConfiguration("model_path"),
                    "device":       LaunchConfiguration("device"),
                },
            ],
        ),
    ])
