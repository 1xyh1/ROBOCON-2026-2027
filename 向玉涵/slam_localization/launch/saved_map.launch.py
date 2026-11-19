"""Open an already saved map without starting mapping or localization."""

from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, OpaqueFunction, Shutdown
from launch_ros.actions import Node
import yaml


ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config" / "saved_map_launch.yaml"
TOOLS = ROOT / "resources/tools" / "map_review"


def start_preview(context):
    with CONFIG.open(encoding="utf-8") as stream:
        config = yaml.load(stream, Loader=yaml.BaseLoader)
    name = config["map_name"]
    if name in ("", ".", "..") or "/" in name or "\\" in name:
        raise ValueError(f"Set map_name to the saved directory name in {CONFIG}")
    directory = (ROOT / config["map_dir"] / name).resolve()
    if not (directory / "map.yaml").is_file():
        raise FileNotFoundError(directory / "map.yaml")
    rviz = config["rviz"].lower()
    if rviz not in ("true", "false"):
        raise ValueError(f"rviz must be true or false in {CONFIG}")

    actions = [
        LogInfo(msg=f"Saved map preview: {directory}"),
        ExecuteProcess(
            cmd=[sys.executable, "-B", str(TOOLS / "preview_map.py"), str(directory)],
            name="saved_map_preview",
            output="screen",
            on_exit=[Shutdown(reason="Saved map publisher exited")],
        ),
    ]
    if rviz == "true":
        actions.append(Node(
            package="rviz2", executable="rviz2", name="saved_map_rviz",
            arguments=["-d", str(TOOLS / "saved_map.rviz")],
            output="screen",
            on_exit=[Shutdown(reason="Saved map RViz closed")],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=start_preview)])
