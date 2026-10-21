"""
sim.launch.py — Terminal 1: Gazebo + TurtleBot3 only.

Brings up gzserver + gzclient with a course world, robot_state_publisher,
and the TurtleBot3 spawn. Worlds and the vendored Gazebo models they
reference both ship in this package (tb3_bringup/worlds, tb3_bringup/models);
GAZEBO_MODEL_PATH is pointed at the installed models so they resolve offline.

Usage:
    export TURTLEBOT3_MODEL=waffle_pi
    ros2 launch tb3_bringup sim.launch.py
    ros2 launch tb3_bringup sim.launch.py world:=warehouse_models

Runs standalone — no SLAM/Nav2/course nodes are started here.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration


# Built-in Gazebo world presets shipped in tb3_bringup/worlds.
# Each entry maps a short alias (the value the user passes via
# `world:=...`) to (world_file, default_x, default_y).
#
# The default spawn pose is auto-applied when `x_pose` / `y_pose` are
# left at their sentinel "AUTO" values. Pass `x_pose:=...`/`y_pose:=...`
# explicitly to override.
#
# Two officially supported worlds for the full semantic navigation stack:
#   - warehouse_models_person: **default**. 6×6 m room with five `person`
#                              figures (four corners + centre), sized so
#                              the LDS-01 LiDAR (3.5 m range) always sees
#                              every wall. Tuned for "go to person N".
#   - warehouse_models:        the 4×6 m room with one of each semantic
#                              target — person, trash can and chair. This
#                              is the world the acceptance run defaults to.
#
# detector_test.world remains on disk for its own dedicated launch
# (detector_test.launch.py) but is not exposed as an alias here.
# Pass an absolute path to use it via this launch.
WORLD_PRESETS = {
    "warehouse_models_person": {
        "file":      "warehouse_models_person.world",
        # (-1.5, 0): 1.5 m from the west wall (well clear of the default
        # Nav2 inflation_radius=0.55 m), 1.5 m from the centre person,
        # and 2.06 m from each of the NW/SW corner persons. Closer poses
        # sat inside the inflation halo and made planner_server refuse
        # the very first goal.
        "default_x": "-1.5",
        "default_y": "0.0",
    },
    "warehouse_models": {
        "file":      "warehouse_semantic_models.world",
        "default_x": "-1.2",
        "default_y": "-1.2",
    },
}

# Used as the fallback when the user passes a custom world (absolute
# path), since we have no way to know its valid spawn region.
_FALLBACK_SPAWN_X = "-1.2"
_FALLBACK_SPAWN_Y = "-1.2"


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    pkg_bringup = get_package_share_directory("tb3_bringup")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    launch_tb3 = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "launch"
    )

    def make_gazebo_actions(context, *_args, **_kwargs):
        """Resolve `world:=...` and per-world spawn defaults, then build
        the Gazebo + TurtleBot3 launch actions accordingly.

        Runs at launch time (not at module import) via OpaqueFunction:
          1. Dict lookup against WORLD_PRESETS needs the resolved string.
          2. `x_pose=AUTO` / `y_pose=AUTO` means "use the per-world
             default" — distinguishing that from a user-provided number
             requires the resolved string, which substitutions alone
             cannot give us.
        """
        raw = LaunchConfiguration("world").perform(context).strip()
        if not raw:
            raw = "warehouse_models_person"

        if raw in WORLD_PRESETS:
            preset = WORLD_PRESETS[raw]
            world_file = os.path.join(pkg_bringup, "worlds", preset["file"])
            preset_x   = preset["default_x"]
            preset_y   = preset["default_y"]
            source = "alias"
        else:
            # Treat as a direct path. We deliberately do not silently
            # fall back to the default if the file is missing — fail
            # loudly so users notice typos.
            world_file = os.path.abspath(os.path.expanduser(raw))
            preset_x   = _FALLBACK_SPAWN_X
            preset_y   = _FALLBACK_SPAWN_Y
            source = "path"

        if not os.path.isfile(world_file):
            valid_aliases = ", ".join(sorted(WORLD_PRESETS.keys()))
            raise FileNotFoundError(
                f"[sim] world={raw!r} could not be resolved. "
                f"Tried as {source}: {world_file}. "
                f"Pass one of the built-in aliases ({valid_aliases}) or an "
                f"absolute path to a .world file."
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
                f"[sim] world={raw!r} → {world_file} ;"
                f" spawn=({x_final}, {y_final})"
            )),
            gzserver, gzclient, rsp, spawn,
        ]

    # The .world files reference vendored Gazebo models via model:// URIs.
    # Prepend the package-shipped models directory so a fresh machine never
    # has to hit the (deprecated, slow) online Gazebo model database.
    set_gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[
            os.path.join(pkg_bringup, "models"),
            ":",
            EnvironmentVariable("GAZEBO_MODEL_PATH", default_value=""),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),

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
            "world",
            default_value="warehouse_models_person",
            description=(
                "Gazebo world: alias (warehouse_models_person | "
                "warehouse_models) or absolute path to a .world file."
            ),
        ),

        # Must come before the Gazebo actions below.
        set_gazebo_model_path,

        OpaqueFunction(function=make_gazebo_actions),
    ])
