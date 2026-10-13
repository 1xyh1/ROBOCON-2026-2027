cmake_minimum_required(VERSION 3.10)
project(br_brain)   #BR的brain部分


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON) 

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(main 
    src/main.cpp 
    src/br_brain/arbitrate_layer.cpp 
    src/br_brain/world_model.cpp 
    #组装者
    src/br_brain/assembler.cpp


    #任务类相关
    src/br_brain/missions/mission_base.cpp 
    src/br_brain/missions/build_earth.cpp
    src/br_brain/missions/enter_field.cpp

    #技能类相关
    src/br_brain/skills/skill_layer.cpp
    src/br_brain/skills/skill_base.cpp
    src/br_brain/skills/skill_chassis.cpp

    #技能类包含的规划器类相关
    src/br_brain/skills/planners/path_planner.cpp




    #utils工具函数
    src/utils/map_loader.cpp


    )

target_include_directories(main PRIVATE include)



ament_target_dependencies(main
    rclcpp              #ros2本体
    std_msgs            #基础消息类型
    geometry_msgs       #几何消息
    nav_msgs            #导航信息
    sensor_msgs         #传感器信息
    ament_index_cpp     
)


install(TARGETS main
    DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY maps DESTINATION share/${PROJECT_NAME})   #地图装进share


ament_package()