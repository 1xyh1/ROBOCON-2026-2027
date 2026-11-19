"""Start live MID-360 loop mapping with optional sensor recording."""

from datetime import datetime
from ipaddress import IPv4Address
import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.actions import LogInfo, OpaqueFunction, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = PROJECT_ROOT / "config"
LIDAR_TOPIC = "/livox/lidar"
IMU_TOPIC = "/livox/imu"
CAMERA_NAME = "camera"
CAMERA_NAMESPACE = "camera"
CAMERA_PREFIX = f"/{CAMERA_NAMESPACE}/{CAMERA_NAME}"
BAG_TOPICS = (
    LIDAR_TOPIC, IMU_TOPIC, "/livox/clock_packets", "/livox/clock_device",
    f"{CAMERA_PREFIX}/color/image_raw",
    f"{CAMERA_PREFIX}/color/camera_info",
    f"{CAMERA_PREFIX}/color/metadata",
    f"{CAMERA_PREFIX}/aligned_depth_to_color/image_raw",
    f"{CAMERA_PREFIX}/depth/metadata",
    "/tf", "/tf_static",
)


def start_mapping(context):
    lio_config = CONFIG_DIR / LaunchConfiguration("config").perform(context)
    livox_config = CONFIG_DIR / LaunchConfiguration("livox_config").perform(context)
    lio_yaml = lio_config.read_bytes()
    livox_json = livox_config.read_bytes()

    parameters = yaml.safe_load(lio_yaml)["/**"]["ros__parameters"]
    # The mapper pairs these frames and uses the fixed LiDAR-to-IMU offset for its rays.
    required = {
        "common": {
            "lid_topic": LIDAR_TOPIC, "imu_topic": IMU_TOPIC,
            "odom_frame": "odom", "body_frame": "mid360_imu", "publish_tf": True,
        },
        "preprocess": {"lidar_type": 1},
        "mapping": {"extrinsic_est_en": False},
        "publish": {"scan_publish_en": True, "scan_bodyframe_pub_en": True},
    }
    for section, values in required.items():
        for key, value in values.items():
            configured = parameters[section].get(key)
            if type(configured) is not type(value) or configured != value:
                raise ValueError(f"{lio_config} requires {section}.{key} = {value!r} for mapping")
    if parameters.get("pcd_save", {}).get("pcd_save_en", False):
        raise ValueError(f"{lio_config} requires pcd_save.pcd_save_en = false for map directory output")

    livox = json.loads(livox_json)
    devices = livox["lidar_configs"]
    if len(devices) != 1:
        raise ValueError(f"{livox_config} must describe one MID-360")
    lidar_ip = str(IPv4Address(devices[0]["ip"]))
    host_ip = str(IPv4Address(livox["MID360"]["host_net_info"]["cmd_data_ip"]))

    map_name = LaunchConfiguration("map_name").perform(context) or datetime.now().strftime("%Y%m%d_%H%M%S")
    if map_name in (".", "..") or "/" in map_name or "\\" in map_name:
        raise ValueError("map_name must be a single directory name")
    map_dir = PROJECT_ROOT / LaunchConfiguration("map_dir").perform(context)
    output = (map_dir / map_name).resolve()
    bringup = Path(get_package_share_directory("robocon_bringup")) / "launch/upper_computer.launch.py"

    record = LaunchConfiguration("record").perform(context) == "true"
    if record:
        bag_path = PROJECT_ROOT / "bag" / map_name
        if bag_path.exists():
            raise FileExistsError(bag_path)
        camera_config = CONFIG_DIR / LaunchConfiguration("camera_config").perform(context)
        camera_yaml = camera_config.read_bytes()
        camera_settings = yaml.safe_load(camera_yaml)
        for key in ("enable_color", "enable_depth", "enable_sync", "align_depth.enable"):
            if camera_settings.get(key) is not True:
                raise ValueError(f"{camera_config} requires {key} = true for RGB-D recording")
        realsense_launch = Path(get_package_share_directory("realsense2_camera")) / "launch/rs_launch.py"

    output.mkdir(parents=True, exist_ok=False)
    lio_snapshot = output / "fast_lio.yaml"
    livox_snapshot = output / "livox.json"
    lio_snapshot.write_bytes(lio_yaml)
    livox_snapshot.write_bytes(livox_json)

    actions = [LogInfo(msg=f"Mapping output {output}")]
    if record:
        bag_path.parent.mkdir(parents=True, exist_ok=True)
        (output / "bag").symlink_to(bag_path, target_is_directory=True)
        camera_snapshot = output / "d435i.yaml"
        camera_snapshot.write_bytes(camera_yaml)
        actions.append(ExecuteProcess(
            cmd=["ros2", "bag", "record", "-s", "sqlite3", "-o", str(bag_path), *BAG_TOPICS],
            name="mapping_bag_recorder",
            output="screen",
            sigterm_timeout="30",
            on_exit=[Shutdown(reason="Mapping bag recorder exited")],
        ))
        # The camera publishes the raw frames used by d435i_time_align_node on replay.
        actions.append(GroupAction(
            scoped=True,
            forwarding=False,
            launch_configurations={
                "config_file": str(camera_snapshot),
                "camera_name": CAMERA_NAME,
                "camera_namespace": CAMERA_NAMESPACE,
            },
            actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(str(realsense_launch)))],
        ))

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(bringup)),
        launch_arguments={
            "with_mid360": "true",
            "with_lio": "true",
            "with_camera": "false",
            "with_rviz": LaunchConfiguration("rviz").perform(context),
            "use_sim_time": "false",
            "mapping_output": str(output),
            "localization_config": "",
            "lio_config_file": str(lio_snapshot),
            "livox_config": str(livox_snapshot),
            "mid360_ip": lidar_ip,
            "mid360_host_ip": host_ip,
        }.items(),
    ))
    return actions


def generate_launch_description():
    # Launch arguments are strings. Keep timestamped map names exactly as written.
    with (CONFIG_DIR / "mapping_launch.yaml").open(encoding="utf-8") as stream:
        defaults = yaml.load(stream, Loader=yaml.BaseLoader)
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=defaults["config"],
                              description="FAST-LIO YAML in config/ or an absolute path"),
        DeclareLaunchArgument("livox_config", default_value=defaults["livox_config"],
                              description="MID-360 JSON in config/ or an absolute path"),
        DeclareLaunchArgument("camera_config", default_value=defaults["camera_config"],
                              description="D435i YAML in config/ or an absolute path for recording"),
        DeclareLaunchArgument("map_dir", default_value=defaults["map_dir"],
                              description="Map root relative to the project, or an absolute path"),
        DeclareLaunchArgument("map_name", default_value=defaults["map_name"],
                              description="New map directory name; defaults to a timestamp"),
        DeclareLaunchArgument("record", default_value=defaults["record"].lower(), choices=["true", "false"],
                              description="Record LiDAR, IMU, clocks and D435i RGB-D under bag/"),
        DeclareLaunchArgument("rviz", default_value=defaults["rviz"].lower(), choices=["true", "false"],
                              description="Open the mapping view in RViz"),
        OpaqueFunction(function=start_mapping),
    ])
