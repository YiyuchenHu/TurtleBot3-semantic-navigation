"""
nav.launch.py — Terminal 2: SLAM Toolbox + Nav2 + RViz.

Starts the Nav2 bringup with slam:=True (which launches slam_toolbox
internally, so no pre-built map file is needed) plus RViz with the course
config. Start this AFTER the simulation (sim.launch.py) is up: SLAM needs
/scan and /clock from Gazebo before it can publish /map, and Nav2's
lifecycle manager waits on /map to finish configuring.

Usage:
    ros2 launch tb3_bringup nav.launch.py
    ros2 launch tb3_bringup nav.launch.py use_rviz:=false

Runs standalone — without the simulation it simply waits (lifecycle
manager logs "Waiting for service ..."), it does not crash.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    pkg_nav2 = get_package_share_directory("nav2_bringup")

    # Nav2 bringup: slam=True makes it launch slam_toolbox internally.
    # map arg is required even with slam=True; provide a dummy path that
    # won't be loaded. params_file must be explicit to avoid empty-path
    # errors from ParameterFile.
    dummy_map = os.path.join(pkg_nav2, "maps", "turtlebot3_world.yaml")
    nav2_params = os.path.join(pkg_nav2, "params", "nav2_params.yaml")

    # Mute the very noisy `worldToMap failed: mx,my: ...` ERROR that
    # planner_server prints whenever the costmap inflation samples one cell
    # past the static-map boundary. It is benign in Nav2 Humble (planning
    # still succeeds and the goal is reached), but it floods the terminal
    # at planning rate.
    #
    # nav2_bringup forwards this single string verbatim as
    # `--ros-args --log-level <value>` to every Nav2 node. ROS 2's
    # `--log-level` supports a per-logger form `<logger>:=<level>`; when
    # passed a logger name that does not exist on the receiving node the
    # rcl logging machinery silently ignores it and the process default
    # stays at INFO. Therefore `planner_server:=fatal`:
    #   • on planner_server  → matches its node logger → silenced.
    #   • on every other Nav2 node (bt_navigator, controller_server, …)
    #     → no such logger → INFO defaults preserved, goal lifecycle and
    #     recoveries still print.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam": "True",
            "map": dummy_map,
            "params_file": nav2_params,
            "autostart": "True",
            "use_composition": "False",
            "log_level": "planner_server:=fatal",
        }.items(),
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare("tb3_bringup"), "rviz", "semantic_nav.rviz"
    ])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="Launch RViz with semantic nav config"),
        nav2,
        rviz_node,
    ])
