"""
full_semantic_nav_la.launch.py — Full semantic navigation stack with
LocateAnything as the ONLY perception model (Phase 2b mapping path).

Identical to full_semantic_nav.launch.py except:
  1. The YOLO detector is NOT started (its code stays in the repo,
     ground rule 7). la_detector_node produces /detector_node/detections
     instead, by calling the grounding container's POST /detect
     (fixed vocabulary, config/la_detector.yaml).
  2. semantic_query_node loads semantic_targets_la.yaml — the registry
     matching the LocateAnything vocabulary (person / bookshelf /
     trash_can / cone) instead of the YOLO one (person / table).
Everything else (Gazebo, Nav2+SLAM, exploration, localizer, memory,
map memory, nav adapter, coordinator, RViz) is byte-identical to the
baseline launch, so YOLO-vs-LA comparisons only change the producer.

Prerequisite: the grounding compose service must be up
(`docker compose ... up -d grounding`; cold start ~76 s — la_detector
logs a throttled warning until /detect answers).

Usage:
    ros2 launch tb3_coordinator full_semantic_nav_la.launch.py
    ros2 topic pub --once /user_command std_msgs/String "data: 'go to the bookshelf'"

R07 defaults: spawn (1, -1) for the person world, stock Humble Nav2
parameters (nav2_params:=... to override), warm-up scan 0.4 rad/s.
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
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# World presets, same shape as the baseline launch's table (duplicated on
# purpose; cross-importing launch files is worse). NOT identical any more:
# the person world's default spawn differs (see the comment below).
WORLD_PRESETS = {
    "warehouse_models_person": {
        "file":      "warehouse_models_person.world",
        # R07: (1, -1) instead of the baseline's (-1.5, 0). The R05
        # jersey barrier sits at x=-2.55 along the west wall, 1.05 m
        # from (-1.5, 0) — inside the runbook §9 ">= 1.2 m from any
        # obstacle" rule; the R06a smoke run spent its first 100 s
        # with goal_assignment reporting "No low-cost sample". (1, -1)
        # is the start R05 B1 (the reference 4-command run) used:
        # 1.41 m from the centre person, 1.41 m from the SE person
        # (2, -2), 1.72 m from the cone (2.4, 0), 1.64 m from the
        # trash can (0, -2.3), 2 m from the east/south walls.
        "default_x": "1.0",
        "default_y": "-1.0",
    },
    "warehouse_models": {
        "file":      "warehouse_semantic_models.world",
        "default_x": "-1.2",
        "default_y": "-1.2",
    },
}

_FALLBACK_SPAWN_X = "-1.2"
_FALLBACK_SPAWN_Y = "-1.2"


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    pkg_fe = get_package_share_directory("tb3_frontier_exploration")
    pkg_det = get_package_share_directory("tb3_detector")
    pkg_loc = get_package_share_directory("tb3_localizer")
    pkg_mem = get_package_share_directory("tb3_memory")
    pkg_nav = get_package_share_directory("tb3_nav_adapter")
    pkg_coord = get_package_share_directory("tb3_coordinator")
    pkg_nav2 = get_package_share_directory("nav2_bringup")

    # ── 1. Gazebo + TurtleBot3 (identical to baseline) ────────────────────
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    launch_tb3 = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "launch"
    )

    def make_gazebo_actions(context, *_args, **_kwargs):
        raw = LaunchConfiguration("world").perform(context).strip()
        if not raw:
            raw = "warehouse_models_person"

        if raw in WORLD_PRESETS:
            preset = WORLD_PRESETS[raw]
            world_file = os.path.join(pkg_fe, "worlds", preset["file"])
            preset_x   = preset["default_x"]
            preset_y   = preset["default_y"]
            source = "alias"
        else:
            world_file = os.path.abspath(os.path.expanduser(raw))
            preset_x   = _FALLBACK_SPAWN_X
            preset_y   = _FALLBACK_SPAWN_Y
            source = "path"

        if not os.path.isfile(world_file):
            valid_aliases = ", ".join(sorted(WORLD_PRESETS.keys()))
            raise FileNotFoundError(
                f"[full_semantic_nav_la] world={raw!r} could not be resolved. "
                f"Tried as {source}: {world_file}. "
                f"Pass one of the built-in aliases ({valid_aliases}) or an "
                f"absolute path to a .world file."
            )

        # Repo-local model variants (person_<color>_shirt, generated by
        # scripts/make_person_variants.py). The installed world file is a
        # symlink into src/, so realpath lands next to the models/ dir.
        models_dir = os.path.normpath(os.path.join(
            os.path.dirname(os.path.realpath(world_file)), "..", "models"
        ))
        model_path_actions = []
        if os.path.isdir(models_dir):
            model_path_actions.append(
                AppendEnvironmentVariable("GAZEBO_MODEL_PATH", models_dir)
            )

        x_raw = LaunchConfiguration("x_pose").perform(context).strip()
        y_raw = LaunchConfiguration("y_pose").perform(context).strip()
        x_final = preset_x if x_raw == "AUTO" else x_raw
        y_final = preset_y if y_raw == "AUTO" else y_raw

        gzserver = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
            ),
            launch_arguments={"world": world_file}.items(),
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
            launch_arguments={"x_pose": x_final, "y_pose": y_final}.items(),
        )

        return [
            LogInfo(msg=(
                f"[full_semantic_nav_la] world={raw!r} → {world_file} ;"
                f" spawn=({x_final}, {y_final})"
            )),
            *model_path_actions,
            gzserver, gzclient, rsp, spawn,
        ]

    # ── 2. Nav2 + SLAM (identical to baseline) ──────────────────────────
    dummy_map = os.path.join(pkg_nav2, "maps", "turtlebot3_world.yaml")
    # R07: back to the stock Humble parameters (the R05 configuration,
    # same file as the baseline launch). R06a step 7 had switched to a
    # 0.3 rad/s angular-limit copy; its one smoke run wedged at the
    # centre person (676 "No valid trajectories" vs 0-2 in the R05
    # logs) and n=1 could not clear the slower DWB sampling as a cause.
    # That file is kept as config/nav2_params_la_slow.yaml — pass it via
    # nav2_params:=<abs path> (plus warmup_angular_speed:=0.3) to try it.
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam": "True",
            "map": dummy_map,
            "params_file": LaunchConfiguration("nav2_params"),
            "autostart": "True",
            "use_composition": "False",
            "log_level": "planner_server:=fatal",
        }.items(),
    )

    # ── 4. Frontier exploration (identical) ───────────────────────────────
    exploration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_fe, "launch", "exploration.launch.py")
        ),
        # R07: start-up rotation scan speed is a launch argument again
        # (default 0.4 rad/s = the R05 value and exploration.launch.py's
        # own default; R06a step 7 had pinned 0.3 here).
        launch_arguments={
            "use_sim_time": use_sim_time,
            "warmup_angular_speed": LaunchConfiguration("warmup_angular_speed"),
        }.items(),
    )

    # ── 5. Semantic perception chain ──────────────────────────────────────
    # THE difference from the baseline: la_detector_node instead of the
    # YOLO detector.launch.py include. Publishes on the same absolute
    # topics, so localizer/memory/RViz need no changes.
    # detector_config selects the la_detector parameter profile
    # (docs/la_runbook.md §7):
    #   default → config/la_detector.yaml (R05 map-and-search: registry
    #     classes + shirt-color expressions all map during exploration)
    #   la_detector_search_first.yaml — R04 demo shape (empty vocabulary)
    #   la_detector_four_class.yaml   — pre-R04 default
    #   la_detector_person_only.yaml  — R02c acceptance demo
    la_detector = Node(
        package="tb3_detector",
        executable="la_detector_node",
        name="la_detector_node",
        parameters=[
            LaunchConfiguration("detector_config"),
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    localizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_loc, "launch", "localizer.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    memory = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mem, "launch", "semantic_memory.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # Query node started directly (not via semantic_query.launch.py) so
    # the LocateAnything registry can be injected — that launch file has
    # no semantic_targets_file argument and stays untouched for the
    # baseline path.
    query = Node(
        package="tb3_query",
        executable="semantic_query_node.py",
        name="semantic_query_node",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("tb3_query"), "config", "semantic_query.yaml"
            ]),
            {
                "use_sim_time": use_sim_time,
                "semantic_targets_file": os.path.join(
                    pkg_fe, "config", "semantic_targets_la.yaml"),
            },
        ],
        output="screen",
    )

    nav_adapter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, "launch", "nav_goal_adapter.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # ── 6. Coordinator (identical) ────────────────────────────────────────
    coordinator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_coord, "launch", "coordinator.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    # ── 7. RViz ──────────────────────────────────────────────────────────
    # rviz_config launch argument (R04): default is the recording layout
    # la_demo.rviz (map view + large detector debug image, side panels
    # hidden); pass the absolute path of semantic_nav.rviz for the full
    # debugging layout.
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true",
                              description="Launch RViz with semantic nav config"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(
                pkg_coord, "rviz", "la_demo.rviz"),
            description=(
                "RViz layout. Default: la_demo.rviz (R04 recording "
                "layout — map view + large detector debug image). Pass "
                "the absolute path of semantic_nav.rviz for the full "
                "debugging layout."
            ),
        ),
        DeclareLaunchArgument(
            "x_pose", default_value="AUTO",
            description=(
                "Robot spawn x in map frame, or 'AUTO' to use the "
                "world preset's recommended pose."
            ),
        ),
        DeclareLaunchArgument(
            "y_pose", default_value="AUTO",
            description=(
                "Robot spawn y in map frame, or 'AUTO' to use the "
                "world preset's recommended pose."
            ),
        ),
        DeclareLaunchArgument(
            "detector_config",
            default_value=os.path.join(pkg_det, "config", "la_detector.yaml"),
            description=(
                "la_detector_node parameter file. Default: the R05 "
                "map-and-search profile (registry classes + shirt-color "
                "expressions all map; unmapped queries still search). "
                "Other profiles: la_detector_search_first.yaml (R04 "
                "demo shape), la_detector_four_class.yaml (pre-R04 "
                "default), la_detector_person_only.yaml (R02c "
                "acceptance)."
            ),
        ),
        DeclareLaunchArgument(
            "nav2_params",
            default_value=os.path.join(pkg_nav2, "params", "nav2_params.yaml"),
            description=(
                "Nav2 parameter file. Default: the stock Humble "
                "nav2_bringup file (R05 configuration, same as the "
                "baseline launch). Option: the absolute path of "
                "tb3_coordinator/config/nav2_params_la_slow.yaml (every "
                "angular limit 0.3 rad/s, R06a step 7)."
            ),
        ),
        DeclareLaunchArgument(
            "warmup_angular_speed", default_value="0.4",
            description=(
                "Start-up rotation scan speed in rad/s (R05: 0.4; pair "
                "0.3 with nav2_params_la_slow.yaml)."
            ),
        ),
        DeclareLaunchArgument(
            "world",
            default_value="warehouse_models_person",
            description=(
                "Gazebo world: alias (warehouse_models_person | "
                "warehouse_models) or absolute path to a .world file."
            ),
        ),

        # Same phased timers as the baseline launch.
        OpaqueFunction(function=make_gazebo_actions),
        TimerAction(period=10.0, actions=[nav2]),

        TimerAction(period=20.0, actions=[exploration]),
        TimerAction(period=15.0, actions=[la_detector]),
        TimerAction(period=15.0, actions=[localizer]),
        TimerAction(period=15.0, actions=[memory]),
        TimerAction(period=15.0, actions=[query]),
        TimerAction(period=15.0, actions=[nav_adapter]),

        TimerAction(period=23.0, actions=[coordinator]),

        TimerAction(period=17.0, actions=[
            Node(
                package="tb3_coordinator",
                executable="semantic_map_memory_node",
                name="semantic_map_memory_node",
                parameters=[
                    PathJoinSubstitution([
                        FindPackageShare("tb3_coordinator"), "config", "coordinator.yaml"
                    ]),
                    {"use_sim_time": use_sim_time},
                ],
                output="screen",
            ),
        ]),

        rviz_node,

        DeclareLaunchArgument("use_runtime_debug", default_value="false",
                              description="Launch semantic runtime debug node"),
        TimerAction(period=17.0, actions=[
            Node(
                package="tb3_coordinator",
                executable="semantic_runtime_debug_node",
                name="semantic_runtime_debug_node",
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(LaunchConfiguration("use_runtime_debug")),
                output="screen",
            ),
        ]),
    ])
