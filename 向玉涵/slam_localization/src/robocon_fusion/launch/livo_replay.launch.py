"""Replay-only experiment. Does not start drivers, robot commands or TF authority."""
import json
import hashlib
import os
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


OUTPUT_TOPICS = (
    "/cloud_registered", "/visualization_marker", "/cloud_visual_sub_map_before",
    "/cloud_effected", "/Laser_map", "/aft_mapped_to_init", "/path", "/planner_normal",
    "/voxels", "/dyn_obj", "/dyn_obj_removed", "/dyn_obj_dbg_hist",
    "/mavros/vision_pose/pose", "/rgb_img", "/LIVO2/imu_propagate", "/planes",
)


def start(context):
    if os.environ.get("ROS_DOMAIN_ID") != "179" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("Replay requires ROS_DOMAIN_ID=179 ROS_LOCALHOST_ONLY=1")
    directory = Path(LaunchConfiguration("config_dir").perform(context)).resolve()
    config = yaml.safe_load((directory / "livo.yaml").read_text())["/**"]["ros__parameters"]
    evidence = json.loads((directory / "evidence.json").read_text())
    for name in ("livo.yaml", "camera.json"):
        if hashlib.sha256((directory / name).read_bytes()).hexdigest() != evidence["generated_sha256"][name]:
            raise RuntimeError(f"Generated replay asset changed: {name}")
    if config["common"]["publish_tf"] or evidence["publish_tf"]:
        raise RuntimeError("No TF publication is permitted by this replay launch")
    debug = LaunchConfiguration("gdb").perform(context) == "true"
    return [
        Node(package="robocon_time_alignment", executable="d435i_time_align_node", output="screen",
             condition=IfCondition(LaunchConfiguration("raw_camera")),
             parameters=[{"use_sim_time": True}], on_exit=[Shutdown(reason="D435i time alignment exited")]),
        Node(package="robocon_fusion", executable="livo_rgb_rectifier", output="screen",
             parameters=[{"camera_file": str(directory / "camera.json"), "use_sim_time": True}],
             on_exit=[Shutdown(reason="RGB adapter exited")]),
        Node(package="fast_livo", executable="fastlivo_mapping", output="screen",
             prefix="gdb -batch -ex 'handle SIGINT nostop noprint pass' -ex run -ex 'thread apply all bt' --args" if debug else "",
             parameters=[str(directory / "livo.yaml"), {"use_sim_time": True}],
             on_exit=[Shutdown(reason="fusion frontend exited")],
             remappings=[(topic, "/fusion/livo" + topic) for topic in OUTPUT_TOPICS]
                        + [("/tf", "/fusion/livo/tf"), ("/tf_static", "/fusion/livo/tf_static")]),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("config_dir"),
                              DeclareLaunchArgument("raw_camera", default_value="false"),
                              DeclareLaunchArgument("gdb", default_value="false"),
                              OpaqueFunction(function=start)])
