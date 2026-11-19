cmake_minimum_required(VERSION 3.8)
project(robocon_global_localization)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(GTSAM REQUIRED)
find_package(PCL 1.12 REQUIRED COMPONENTS common filters io kdtree registration)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(robocon_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(robocon_gicp
  src/gicp_matcher.cpp
  src/global_search.cpp
  third_party/3d_bbs/bbs3d/src/cpu_bbs3d/bbs3d.cpp
  third_party/3d_bbs/bbs3d/src/cpu_bbs3d/voxelmaps.cpp)
target_compile_features(robocon_gicp PUBLIC cxx_std_17)
target_include_directories(robocon_gicp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS})
target_include_directories(robocon_gicp PRIVATE third_party/3d_bbs/bbs3d/include)
target_link_libraries(robocon_gicp ${PCL_LIBRARIES} OpenMP::OpenMP_CXX)
target_compile_definitions(robocon_gicp PUBLIC ${PCL_DEFINITIONS})

add_executable(global_localizer_node src/global_localizer_node.cpp)
target_compile_features(global_localizer_node PUBLIC cxx_std_17)
target_link_libraries(global_localizer_node robocon_gicp)
ament_target_dependencies(global_localizer_node builtin_interfaces geometry_msgs nav_msgs pcl_conversions
  rclcpp rclcpp_action robocon_interfaces sensor_msgs std_msgs tf2_ros)

install(TARGETS robocon_gicp global_localizer_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES third_party/3d_bbs/LICENSE DESTINATION share/${PROJECT_NAME}/licenses/3d_bbs)

add_library(robocon_pose_graph
  src/loop_closure.cpp
  src/static_map.cpp
  third_party/fastlio2_ros2_pgo/src/pgos/commons.cpp
  third_party/fastlio2_ros2_pgo/src/pgos/simple_pgo.cpp)
target_compile_features(robocon_pose_graph PUBLIC cxx_std_17)
target_include_directories(robocon_pose_graph PUBLIC third_party/fastlio2_ros2_pgo/src/pgos)
target_link_libraries(robocon_pose_graph robocon_gicp gtsam)
add_executable(map_builder_node src/map_builder_node.cpp)
target_link_libraries(map_builder_node robocon_pose_graph)
ament_target_dependencies(map_builder_node geometry_msgs nav_msgs pcl_conversions rclcpp
  sensor_msgs std_srvs tf2_ros)
install(TARGETS robocon_pose_graph map_builder_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(FILES third_party/fastlio2_ros2_pgo/LICENSE
  DESTINATION share/${PROJECT_NAME}/licenses/fastlio2_ros2_pgo)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_gicp_matcher test/test_gicp_matcher.cpp)
  target_link_libraries(test_gicp_matcher robocon_gicp)
  ament_add_gtest(test_pose_graph test/test_pose_graph.cpp)
  target_link_libraries(test_pose_graph robocon_pose_graph)
endif()

ament_package()
