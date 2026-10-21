"""
full_stack.launch.py — Terminals 1-5 in one process (convenience shell).

The RECOMMENDED way to run the course is the six-terminal flow in the
README Quick start (sim / nav / backend / localizer / detector as
separate `ros2 launch` commands, so each subsystem has its own logs and
can be restarted alone). This file exists for the cases where one command
is preferable (demos, quick smoke tests, the grader's batch run).

It is a thin shell: it only includes the five standalone sub-launches,
with minimal fixed delays standing in for the "wait until ready" steps a
human performs in the six-terminal flow.

Usage:
    export TURTLEBOT3_MODEL=waffle_pi
    ros2 launch tb3_bringup full_stack.launch.py

    # Then in another terminal:
    ros2 topic pub --once /user_command std_msgs/String "data: 'go to person 0'"

All arguments are forwarded to the relevant sub-launch: `world`,
`x_pose`, `y_pose` (sim), `use_rviz` (nav), `use_runtime_debug`
(backend), `use_sim_time` (all).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    bringup_launch = os.path.join(
        get_package_share_directory("tb3_bringup"), "launch")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch, "sim.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "world": LaunchConfiguration("world"),
            "x_pose": LaunchConfiguration("x_pose"),
            "y_pose": LaunchConfiguration("y_pose"),
        }.items(),
    )

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch, "nav.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )

    localizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch, "localizer.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch, "detector.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    backend = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_launch, "backend.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_runtime_debug": LaunchConfiguration("use_runtime_debug"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="Launch RViz with semantic nav config"),
        DeclareLaunchArgument("use_runtime_debug", default_value="false",
                              description="Launch semantic runtime debug node"),
        # Forwarded to sim.launch.py, which resolves aliases and the
        # AUTO spawn pose (see WORLD_PRESETS there).
        DeclareLaunchArgument("world", default_value="warehouse_models_person",
                              description="Gazebo world: alias or absolute path"),
        DeclareLaunchArgument("x_pose", default_value="AUTO"),
        DeclareLaunchArgument("y_pose", default_value="AUTO"),

        # Fixed delays replace the human "wait until ready" of the
        # six-terminal flow:
        #   t=0   sim      — Gazebo needs no one.
        #   t=10  nav      — Nav2 (autostart) needs Gazebo up so SLAM can
        #                    publish /map before planner_server configures;
        #                    10 s absorbs CPU jitter (5 s made
        #                    lifecycle_manager hang on "Waiting for service
        #                    planner_server/get_state...").
        #   t=15  detector + localizer — camera/scan topics exist by then.
        #   t=20  backend  — coordinator + exploration want Nav2's
        #                    navigate_to_pose action to be answering.
        sim,
        TimerAction(period=10.0, actions=[nav]),
        TimerAction(period=15.0, actions=[detector]),
        TimerAction(period=15.0, actions=[localizer]),
        TimerAction(period=20.0, actions=[backend]),
    ])
