"""
gazebo_sim.launch.py — Gazebo Classic + TurtleBot3 only (no Nav2, no
perception, no exploration). Layer 1 of the manual bring-up in
docs/la_runbook.md §4.1; everything else is started per node by hand.

    ros2 launch tb3_frontier_exploration gazebo_sim.launch.py
    ros2 launch tb3_frontier_exploration gazebo_sim.launch.py world:=warehouse_models x_pose:=-1.2 y_pose:=-1.2
    ros2 launch tb3_frontier_exploration gazebo_sim.launch.py world:=/abs/path/my.world gui:=false

Arguments:
    world   alias (warehouse_models_person | warehouse_models | detector_test)
            or an absolute path to a .world file      [warehouse_models_person]
    x_pose, y_pose   robot spawn in the world frame    [1.0, -1.0]
    gui     start gzclient                             [true]
    use_sim_time                                       [true]

The repo's models/ dir (person_<colour>_shirt variants) is appended to
GAZEBO_MODEL_PATH for gzserver, so the world's coloured persons resolve.
Requires TURTLEBOT3_MODEL in the environment (the sim container sets
waffle_pi).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

WORLDS = {
    "warehouse_models_person": "warehouse_models_person.world",
    "warehouse_models": "warehouse_semantic_models.world",
    "detector_test": "detector_test.world",
}


def _setup(context, *args, **kwargs):
    pkg_fe = get_package_share_directory("tb3_frontier_exploration")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    launch_tb3 = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "launch")

    raw = LaunchConfiguration("world").perform(context).strip()
    if raw in WORLDS:
        world_file = os.path.join(pkg_fe, "worlds", WORLDS[raw])
    else:
        world_file = os.path.abspath(os.path.expanduser(raw))
    if not os.path.isfile(world_file):
        raise FileNotFoundError(
            "[gazebo_sim] world=%r not found (aliases: %s, or an absolute path)"
            % (raw, ", ".join(sorted(WORLDS))))
    # installed worlds/ is a symlink into src/, so the models/ dir sits next to it
    models_dir = os.path.normpath(os.path.join(
        os.path.dirname(os.path.realpath(world_file)), "..", "models"))

    use_sim_time = LaunchConfiguration("use_sim_time")
    actions = [LogInfo(msg="[gazebo_sim] world=%s  spawn=(%s, %s)" % (
        world_file,
        LaunchConfiguration("x_pose").perform(context),
        LaunchConfiguration("y_pose").perform(context)))]
    if os.path.isdir(models_dir):
        actions.append(AppendEnvironmentVariable("GAZEBO_MODEL_PATH", models_dir))
    actions += [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")),
            launch_arguments={"world": world_file}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")),
            condition=IfCondition(LaunchConfiguration("gui"))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_tb3, "robot_state_publisher.launch.py")),
            launch_arguments={"use_sim_time": use_sim_time}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_tb3, "spawn_turtlebot3.launch.py")),
            launch_arguments={
                "x_pose": LaunchConfiguration("x_pose"),
                "y_pose": LaunchConfiguration("y_pose")}.items()),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="warehouse_models_person",
                              description="alias (%s) or absolute .world path"
                              % " | ".join(sorted(WORLDS))),
        DeclareLaunchArgument("x_pose", default_value="1.0"),
        DeclareLaunchArgument("y_pose", default_value="-1.0"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=_setup),
    ])
