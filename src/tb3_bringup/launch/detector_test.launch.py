"""
detector_test.launch.py — Standalone: Gazebo with detector_test.world.

Not part of the six-terminal flow; this is the Stage-1 YOLO self-test
world from INSTRUCTIONS Step 4.

Usage:
    export TURTLEBOT3_MODEL=burger   # or waffle / waffle_pi
    ros2 launch tb3_bringup detector_test.launch.py

Optional overrides:
    ros2 launch tb3_bringup detector_test.launch.py \
        x_pose:=0.0 y_pose:=0.0
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


def generate_launch_description():
    pkg_bringup = get_package_share_directory("tb3_bringup")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    launch_tb3 = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "launch"
    )

    world = os.path.join(pkg_bringup, "worlds", "detector_test.world")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    # Robot spawns at origin facing +X so all test objects are directly ahead.
    x_pose = LaunchConfiguration("x_pose", default="0.0")
    y_pose = LaunchConfiguration("y_pose", default="0.0")

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        ),
    )

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_tb3, "robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_tb3, "spawn_turtlebot3.launch.py")
        ),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("x_pose", default_value="0.0",
                              description="TB3 spawn X (world frame)"),
        DeclareLaunchArgument("y_pose", default_value="0.0",
                              description="TB3 spawn Y (world frame)"),
        # Vendored Gazebo models (model:// URIs in the .world files).
        SetEnvironmentVariable(
            name="GAZEBO_MODEL_PATH",
            value=[
                os.path.join(pkg_bringup, "models"),
                ":",
                EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
            ],
        ),
        gzserver,
        gzclient,
        rsp,
        spawn,
    ])
