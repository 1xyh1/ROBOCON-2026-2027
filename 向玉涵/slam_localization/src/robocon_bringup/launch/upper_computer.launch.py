"""No-motion upper-computer framework; hardware inputs are explicitly enabled.

Map localization is supplied when its measured-asset parameter file is selected.
No base/arm executor is supplied by this launch.
It does not arm the mission or publish an artificial start/stop acknowledgement.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import yaml


def managed_localization(context):
    """Give the frontend, map matcher and executive one shared process epoch."""
    with_lio = IfCondition(LaunchConfiguration('with_lio')).evaluate(context)
    with_camera = IfCondition(LaunchConfiguration('with_camera')).evaluate(context)
    standalone = IfCondition(LaunchConfiguration('standalone_localization')).evaluate(context)
    localization_file = LaunchConfiguration('localization_config').perform(context)
    mapping_output = LaunchConfiguration('mapping_output').perform(context)
    if mapping_output and localization_file:
        raise ValueError('Mapping and fixed-map localization use separate map->odom owners')
    if mapping_output and not with_lio:
        raise ValueError('Mapping requires the managed LIO frontend')
    if localization_file and not with_lio:
        raise ValueError('Map localization requires the managed LIO frontend')
    if standalone and not localization_file:
        raise ValueError('Standalone localization requires a fixed map configuration')
    checkpoint = Path(LaunchConfiguration('checkpoint_path').perform(context))
    global_params, executive_params = {}, {}
    if localization_file:
        document = yaml.safe_load(Path(localization_file).read_text())
        global_params = document['global_localizer']['ros__parameters']
        if standalone:
            if (not global_params['map_id'] or not global_params['calibration_id']
                    or not global_params['floor'] or not Path(global_params['map_pcd']).is_file()):
                raise ValueError('Standalone map and calibration identities are required')
        else:
            executive_params = document['mission_executive']['ros__parameters']
            if (not global_params['map_id'] or not global_params['calibration_id']
                    or not Path(global_params['map_pcd']).is_file()
                    or executive_params['expected_map_id'] != global_params['map_id']
                    or executive_params['expected_calibration_id'] != global_params['calibration_id']
                    or executive_params['initial_floor'] != global_params['floor']):
                raise ValueError('Map/executive assets or identities do not match')
            from robocon_core.runtime_assets import LOCALIZATION_LIMITS
            for global_key, executive_key in (
                ('maximum_rmse_m', 'maximum_rmse_m'),
                ('minimum_overlap', 'minimum_overlap_ratio'),
                ('minimum_3d_geometry_ratio', 'minimum_geometry_ratio'),
            ):
                global_value = global_params.get(global_key, LOCALIZATION_LIMITS[global_key])
                executive_value = executive_params.get(executive_key, LOCALIZATION_LIMITS[global_key])
                if global_value != executive_value:
                    raise ValueError(f'Localization quality threshold mismatch: {global_key}/{executive_key}')
                global_params[global_key] = executive_params[executive_key] = global_value
    epoch = -1
    actions = []
    if with_lio:
        from robocon_localization.odom_session import allocate_odom_epoch
        epoch = allocate_odom_epoch(checkpoint.parent / 'odom_epoch.json')
        executable = Path(get_package_prefix('fast_lio')) / 'lib/fast_lio/fastlio_mapping'
        actions.append(Node(
            package='robocon_localization', executable='lio_session_supervisor', output='screen',
            parameters=[{'frontend_executable': str(executable),
                         'config_file': LaunchConfiguration('lio_config_file').perform(context),
                         'odom_epoch': epoch}],
            on_exit=[Shutdown(reason='Managed LIO session ended')]))
    if localization_file:
        actions.append(Node(
            package='robocon_global_localization', executable='global_localizer_node',
            name='global_localizer', output='screen', parameters=[global_params, {
                'odom_epoch': epoch, 'publish_tf': True, 'continuous_enabled': True}],
            on_exit=[Shutdown(reason='Global localizer exited')]))
        if standalone:
            auto_params = {key: global_params[key] for key in ('map_id', 'calibration_id', 'floor')}
            auto_params['odom_epoch'] = epoch
            if 'maximum_scan_age_s' in global_params:
                auto_params['maximum_scan_age_s'] = global_params['maximum_scan_age_s']
            actions.append(Node(
                package='robocon_localization', executable='auto_relocalize_node',
                output='screen', parameters=[auto_params],
                on_exit=[Shutdown(reason='Automatic map recovery exited')]))
            return actions
    if mapping_output:
        lio_config = Path(LaunchConfiguration('lio_config_file').perform(context))
        lio = yaml.safe_load(lio_config.read_text())['/**']['ros__parameters']
        lidar_origin = [float(coordinate) for coordinate in lio['mapping']['extrinsic_T']]
        actions.append(Node(package='robocon_global_localization', executable='map_builder_node',
                            output='screen', sigterm_timeout='65',
                            parameters=[{'output_dir': mapping_output,
                                                          'lidar_origin_body': lidar_origin}],
                            on_exit=[Shutdown(reason='Map builder exited')]))
        return actions
    required_clocks = ['mid360_ptp_clock']
    if with_camera:
        required_clocks.append('d435i_color_clock')
    if with_lio:
        actions.append(Node(package='robocon_localization', executable='lio_quality_node', output='screen',
                            parameters=[{'initial_floor': global_params.get('floor', 'ground')}]))
    actions.append(Node(
        package='robocon_executive', executable='mission_executive_node', output='screen',
        parameters=[executive_params, {'checkpoint_path': str(checkpoint),
            'match_id': LaunchConfiguration('match_id'), 'expected_odom_epoch': epoch,
            'required_time_sensors': required_clocks}],
        on_exit=[Shutdown(reason='Mission executive exited')]))
    return actions


def visualization(context):
    if not IfCondition(LaunchConfiguration("with_rviz")).evaluate(context):
        return []
    if IfCondition(LaunchConfiguration("standalone_localization")).evaluate(context):
        view = "relocalization"
    else:
        view = ("mapping" if LaunchConfiguration("mapping_output").perform(context)
                else "map" if LaunchConfiguration("localization_config").perform(context) else "lio")
    share = Path(get_package_share_directory("robocon_bringup"))
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / "launch/visualization.launch.py")),
        launch_arguments={"view": view, "use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
    )]


def generate_launch_description():
    package_share = Path(get_package_share_directory("robocon_bringup"))
    state_home = Path(os.environ.get(
        "ROBOCON_STATE_DIR", str(Path.home() / ".local/state/robocon2027")))
    camera_launch = PathJoinSubstitution([FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"])
    declarations = [
        DeclareLaunchArgument("with_camera", default_value="false"),
        DeclareLaunchArgument("with_mid360", default_value="false"),
        DeclareLaunchArgument("with_lio", default_value="false"),
        DeclareLaunchArgument("with_rviz", default_value="false", description="Open the passive localization viewer on this desktop"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("mapping_output", default_value="", description="Build and save a new map here instead of loading a fixed map"),
        DeclareLaunchArgument("localization_config", default_value="", description="Fixed map parameter file"),
        DeclareLaunchArgument("standalone_localization", default_value="false",
                              description="Run automatic fixed map localization without a mission session"),
        DeclareLaunchArgument("lio_config_file", default_value=str(package_share / "config/fast_lio_mid360.yaml")),
        DeclareLaunchArgument("ptp_verified", default_value="false", description="Deprecated; online evidence is mandatory"),
        DeclareLaunchArgument("ptp_utc_offset_s", default_value="37"),
        DeclareLaunchArgument("camera_serial", default_value="", description="D435i SDK serial; prefix '_' when passing digits through ROS launch"),
        DeclareLaunchArgument("mid360_ip", default_value="192.168.1.139"),
        DeclareLaunchArgument("mid360_host_ip", default_value="0.0.0.0"),
        DeclareLaunchArgument("livox_config", default_value=""),
        DeclareLaunchArgument("checkpoint_path", default_value=str(state_home / "checkpoint.json")),
        DeclareLaunchArgument("match_id", default_value="practice"),
    ]
    hardware = [
        GroupAction(
            actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(camera_launch))],
            condition=IfCondition(LaunchConfiguration("with_camera")),
            scoped=True, forwarding=False,
            launch_configurations={
                "config_file": str(package_share / "config/d435i.yaml"),
                "serial_no": LaunchConfiguration("camera_serial"),
            },
        ),
        Node(
            package="livox_ros_driver2", executable="livox_ros_driver2_node",
            name="livox_lidar_publisher", output="screen",
            condition=IfCondition(LaunchConfiguration("with_mid360")),
            additional_env={
                "ROBOCON_LIVOX_PTP_TAI_UTC_OFFSET_S": LaunchConfiguration("ptp_utc_offset_s"),
                "ROBOCON_LIVOX_REQUIRE_PTP": "1",
            },
            parameters=[{
                "xfer_format": 1, "multi_topic": 0, "data_src": 0,
                "publish_freq": 10.0, "output_data_type": 0,
                "frame_id": "mid360_link",
                "user_config_path": LaunchConfiguration("livox_config"),
            }],
        ),
    ]
    algorithms = [
        Node(package=package, executable=executable, output="screen",
             condition=IfCondition(LaunchConfiguration("with_camera")))
        for package, executable in (
            ("robocon_time_alignment", "d435i_time_align_node"),
            ("robocon_perception", "rgbd_cube_node"),
            ("robocon_perception", "color_start_node"),
        )
    ]
    algorithms.append(Node(
        package="robocon_time_alignment", executable="mid360_stamp_monitor_node",
        output="screen", parameters=[{"ptp_utc_offset_s": ParameterValue(
            LaunchConfiguration("ptp_utc_offset_s"), value_type=int,
        )}],
    ))
    algorithms.append(Node(
        package="robocon_time_alignment", executable="mid360_clock_query_node",
        condition=IfCondition(LaunchConfiguration("with_mid360")), output="screen",
        parameters=[{
            "lidar_ip": LaunchConfiguration("mid360_ip"),
            "host_ip": LaunchConfiguration("mid360_host_ip"),
        }],
    ))
    algorithms.append(OpaqueFunction(function=managed_localization))
    algorithms.append(OpaqueFunction(function=visualization))
    return LaunchDescription(declarations + [SetParameter(
        name='use_sim_time', value=ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    )] + hardware + algorithms)
