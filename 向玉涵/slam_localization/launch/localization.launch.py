"""Start live MID-360 fixed-map localization and optional RViz."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from robocon_core.runtime_assets import standalone_localization_arguments
import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = PROJECT_ROOT / "config"


def start_localization(context):
    map_name = LaunchConfiguration("map_name").perform(context)
    if not map_name or map_name in (".", "..") or "/" in map_name or "\\" in map_name:
        raise ValueError("map_name must be a single directory name")
    folder = (PROJECT_ROOT / LaunchConfiguration("map_dir").perform(context) / map_name).resolve()
    config = LaunchConfiguration("config").perform(context)
    livox = LaunchConfiguration("livox_config").perform(context)
    state = Path(os.environ.get("ROBOCON_STATE_DIR") or PROJECT_ROOT / "runtime-state").resolve()
    arguments = standalone_localization_arguments(
        folder, CONFIG_DIR, state,
        lio_config=CONFIG_DIR / config if config else None,
        livox_config=CONFIG_DIR / livox if livox else None,
    )
    arguments.update({
        "with_mid360": "true", "with_lio": "true", "with_camera": "false",
        "with_rviz": LaunchConfiguration("rviz").perform(context),
        "use_sim_time": "false", "mapping_output": "",
    })
    bringup = Path(get_package_share_directory("robocon_bringup")) / "launch/upper_computer.launch.py"
    return [LogInfo(msg=f"Localization map {folder}"), IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup)), launch_arguments=arguments.items(),
    )]


def generate_launch_description():
    # Launch arguments are strings. Keep timestamped map names exactly as written.
    with (CONFIG_DIR / "localization_launch.yaml").open(encoding="utf-8") as stream:
        defaults = yaml.load(stream, Loader=yaml.BaseLoader)
    return LaunchDescription([
        DeclareLaunchArgument("map_name", default_value=defaults["map_name"],
                              description="Name of a saved map directory"),
        DeclareLaunchArgument("map_dir", default_value=defaults["map_dir"],
                              description="Map root relative to the project, or an absolute path"),
        DeclareLaunchArgument("config", default_value=defaults["config"],
                              description="FAST-LIO YAML in config/ or an absolute path; defaults to the map snapshot"),
        DeclareLaunchArgument("livox_config", default_value=defaults["livox_config"],
                              description="Livox JSON in config/ or an absolute path; defaults to the map profile"),
        DeclareLaunchArgument("rviz", default_value=defaults["rviz"].lower(), choices=["true", "false"],
                              description="Open the fixed-map localization view in RViz"),
        OpaqueFunction(function=start_localization),
    ])
