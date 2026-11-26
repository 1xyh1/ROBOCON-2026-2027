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

find_package(rosidl_default_generators REQUIRED)

find_package(yaml-cpp REQUIRED)




add_executable(main 
    src/main.cpp 
    src/br_brain/arbitrate_layer.cpp 
    src/br_brain/world_model.cpp 
    src/br_brain/mission_strategy.cpp
    #组装者
    src/br_brain/assembler.cpp


    #任务类相关
    src/br_brain/missions/mission_base.cpp 
    src/br_brain/missions/build_earth.cpp
    src/br_brain/missions/enter_field.cpp
    src/br_brain/missions/fetch_earth.cpp
    src/br_brain/missions/fetch_sky.cpp
    src/br_brain/missions/fetch_mustika.cpp
    src/br_brain/missions/build_sky.cpp
    src/br_brain/missions/flip_sky.cpp
    src/br_brain/missions/mustika.cpp
    src/br_brain/missions/debug.cpp


    #技能基类类相关
    src/br_brain/skills/skill_layer.cpp
    src/br_brain/skills/skill_base.cpp
        #底盘
        src/br_brain/skills/chassis/skill_chassis.cpp
        src/br_brain/skills/chassis/path_planner.cpp

    
    
    
    
    #动作基类相关 
    src/br_brain/action/action.cpp
    src/br_brain/action/action_sequence.cpp
    
        #导航动作有关
        src/br_brain/action/navigate/navigate_action.cpp
        src/br_brain/action/navigate/route_planner.cpp



        #抓取动作有关
        src/br_brain/action/fetch/fetch_action.cpp





    #utils工具函数
    src/br_brain/utils/config_loaders/global_loader.cpp
    src/br_brain/utils/config_loaders/map_loader.cpp
    src/br_brain/utils/config_loaders/field_loader.cpp
  


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
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})


#ros2自定义消息部分
rosidl_generate_interfaces(${PROJECT_NAME} 
    "msg/SpotState.msg"
    "msg/TransferState.msg"

)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(main "${cpp_typesupport_target}")
target_link_libraries(main yaml-cpp)
# message(STATUS "yaml-cpp found at: ${yaml-cpp_DIR}")



ament_package()