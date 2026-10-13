import os
from launch.actions import ExecuteProcess, IncludeLaunchDescription

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node




def generate_launch_description():

    #各包share目录的绝对路径 如install/br_sim/share/br_sim
    pkg_br_sim = get_package_share_directory('br_sim')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3 = get_package_share_directory('turtlebot3_gazebo')


    #场地world文件路径
    # world_path = os.path.join(pkg_br_sim, 'worlds', 'field_simplified.world')
    world_path = os.path.join(pkg_br_sim, 'worlds', 'field_empty.world')


    #读TB3机器人描述URDF 内容作为字符串
    urdf_path = os.path.join(pkg_br_sim, 'urdf', 'br_tb3_burger.urdf')
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()


    #起Gazebo 加载我们的场地
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    #起Gazebo图形界面 裸起
    #模型库指向本机空端口 禁掉启动时的在线模型库同步(国外服务器不可达时会挂死在Preparing your world)
    gzclient = ExecuteProcess(
    cmd=['gzclient', '--verbose'],
    additional_env={
        'GAZEBO_MODEL_DATABASE_URI': 'http://127.0.0.1:1',
        'GAZEBO_MODEL_PATH': '/usr/share/gazebo-11/models:/opt/ros/humble/share',
        'GAZEBO_RESOURCE_PATH': '/usr/share/gazebo-11:/opt/ros/humble/share',
    },
    output='screen'
    )


    #发布机器人描述的节点
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': True, 'robot_description': robot_desc}],
    )


    #把机器人注入场地 起始区(-4.5, 4) 朝向+x
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'br_tb3',
               '-topic', 'robot_description',
               '-x', '-4.5', '-y', '4'],
        output='screen',
    )


    ld = LaunchDescription()
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity)
    return ld
