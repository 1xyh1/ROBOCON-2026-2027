"""Display existing localization data without starting sensors or estimators."""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start_viewer(context):
    view = LaunchConfiguration("view").perform(context)
    preset, fixed_frame = {
        "lio": ("lio", "odom"),
        "map": ("map", "map"),
        "relocalization": ("relocalization", "map"),
        "mapping": ("mapping", "map"),
        "fusion": ("fusion", "livo_odom"),
    }[view]
    configuration = Path(__file__).resolve().parents[1] / "rviz" / f"{preset}.rviz"
    remappings = []
    if view == "fusion":
        remappings = [("/tf", "/fusion/livo/tf"),
                      ("/tf_static", "/fusion/livo/tf_static")]
    return [Node(
        package="rviz2", executable="rviz2", name=f"robocon_rviz_{view}",
        output="screen", arguments=["-d", str(configuration), "-f", fixed_frame],
        parameters=[{"use_sim_time": ParameterValue(
            LaunchConfiguration("use_sim_time"), value_type=bool)}],
        remappings=remappings,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("view", default_value="lio", choices=["lio", "map", "mapping", "fusion", "relocalization"],
                              description="FAST-LIO local, GICP map frame, or isolated FAST-LIVO2"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=start_viewer),
    ])
