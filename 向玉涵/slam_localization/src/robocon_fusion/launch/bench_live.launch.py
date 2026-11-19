"""Experimental sensors + LIVO only. No mission, motion executor or TF authority."""
import hashlib
import json
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    if os.environ.get("ROS_DOMAIN_ID") != "178" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("Live bench requires ROS_DOMAIN_ID=178 ROS_LOCALHOST_ONLY=1")
    directory = Path(LaunchConfiguration("config_dir").perform(context)).resolve()
    repo = Path(LaunchConfiguration("repo").perform(context)).resolve()
    evidence = json.loads((directory / "evidence.json").read_text())
    for name in ("livo.yaml", "camera.json"):
        if hashlib.sha256((directory / name).read_bytes()).hexdigest() != evidence["generated_sha256"][name]:
            raise RuntimeError("Changed experimental configuration: " + name)
    params = yaml.safe_load((directory / "livo.yaml").read_text())["/**"]["ros__parameters"]
    if params["common"]["publish_tf"]:
        raise RuntimeError("Bench cannot publish production TF")
    camera = json.loads((directory / "camera.json").read_text())
    share = Path(get_package_share_directory("robocon_bringup"))
    camera_launch = Path(get_package_share_directory("realsense2_camera")) / "launch/rs_launch.py"
    outputs = ("cloud_registered", "visualization_marker", "cloud_visual_sub_map_before", "cloud_effected",
               "Laser_map", "aft_mapped_to_init", "path", "planner_normal", "voxels", "dyn_obj",
               "dyn_obj_removed", "dyn_obj_dbg_hist", "mavros/vision_pose/pose", "rgb_img",
               "LIVO2/imu_propagate", "planes")
    nodes = [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(camera_launch)), launch_arguments={
            "config_file": str(share / "config/d435i.yaml"),
            "serial_no": "_" + camera["camera_serial"], "publish_tf": "false"}.items()),
        Node(package="livox_ros_driver2", executable="livox_ros_driver2_node", output="screen",
             additional_env={"ROBOCON_LIVOX_PTP_TAI_UTC_OFFSET_S": "37", "ROBOCON_LIVOX_REQUIRE_PTP": "1"},
             parameters=[{"xfer_format": 1, "multi_topic": 0, "data_src": 0, "publish_freq": 10.0,
                          "output_data_type": 0, "frame_id": "mid360_link",
                          "user_config_path": str(repo / "config/livox_mid360_mini_host.json")}]),
        Node(package="robocon_time_alignment", executable="d435i_time_align_node", output="screen"),
        Node(package="robocon_time_alignment", executable="mid360_stamp_monitor_node", output="screen",
             parameters=[{"ptp_utc_offset_s": 37}]),
        Node(package="robocon_time_alignment", executable="mid360_clock_query_node", output="screen",
             parameters=[{"lidar_ip": "192.168.1.139", "host_ip": "192.168.1.2"}]),
        Node(package="robocon_fusion", executable="livo_rgb_rectifier", output="screen",
             parameters=[{"camera_file": str(directory / "camera.json"), "use_sim_time": False}],
             on_exit=[Shutdown(reason="RGB adapter exited")]),
        Node(package="fast_livo", executable="fastlivo_mapping", output="screen",
             parameters=[str(directory / "livo.yaml"), {"use_sim_time": False}],
             remappings=[("/" + name, "/fusion/livo/" + name) for name in outputs]
                        + [("/tf", "/fusion/livo/tf"), ("/tf_static", "/fusion/livo/tf_static")],
             on_exit=[Shutdown(reason="LIVO exited")]),
    ]
    if LaunchConfiguration("reference_lio").perform(context) == "true":
        nodes.append(Node(package="fast_lio", executable="fastlio_mapping", name="bench_reference_lio",
                          output="screen", parameters=[str(share / "config/fast_lio_mid360.yaml"), {
                              "common.publish_tf": False, "common.odom_frame": "bench_lio_odom",
                              "common.body_frame": "bench_lio_imu"}],
                          remappings=[("/Odometry", "/bench/reference/odom"),
                                      ("/cloud_registered_body", "/bench/reference/cloud_body"),
                                      ("/cloud_registered", "/bench/reference/cloud"),
                                      ("/path", "/bench/reference/path")]))
    return nodes


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("config_dir"), DeclareLaunchArgument("repo"),
                              DeclareLaunchArgument("reference_lio", default_value="false"),
                              OpaqueFunction(function=start)])
