cmake_minimum_required(VERSION 3.10)
project(br_sim)


find_package(ament_cmake REQUIRED)


#资源包 不编译代码 只把launch和worlds装到share目录
install(DIRECTORY
    launch
    worlds
    urdf
    DESTINATION share/${PROJECT_NAME}
)


ament_package()
