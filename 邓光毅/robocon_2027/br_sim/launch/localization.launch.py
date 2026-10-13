import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    #地图随大脑走 读br_brain安装空间里的maps
    pkg_br_brain = get_package_share_directory('br_brain')
    map_yaml = os.path.join(pkg_br_brain, 'maps', 'field_sim.yaml')     #获取地图yaml文件

    #发布/map话题 内容=field_sim栅格
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'yaml_filename': map_yaml,
        }],
    )

    #粒子滤波定位 订阅/scan+/map 输出/amcl_pose和map->odom那条TF边
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'scan_topic': 'scan',
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'global_frame_id': 'map',
            'set_initial_pose': True,      #启动即用下面的初值
            'initial_pose': {
                'x': -4.5,
                'y': 4.0,
                'z': 0.0,
                'yaw': 0.0,
            },
            'laser_model_type': 'likelihood_field',
            'laser_min_range': 0.12,       #与URDF里雷达参数一致
            'laser_max_range': 8.0,
            'max_particles': 2000,
            'min_particles': 500,
            'tf_broadcast': True,
        }],
    )
    #输入三样: /scan(实测这一眼) /map(地图,用来预测"站在这该看到啥") TF odom→base(这拍动了多少)
    #输出两样: /amcl_pose(定位结果)、TF map→odom(校正边)。



    #生命周期管家 按序把map_server和amcl从unconfigured拉到active
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    ld = LaunchDescription()
    ld.add_action(map_server)
    ld.add_action(amcl)
    ld.add_action(lifecycle_manager)
    return ld
