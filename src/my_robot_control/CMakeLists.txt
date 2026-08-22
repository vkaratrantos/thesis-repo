cmake_minimum_required(VERSION 3.8)
project(my_robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(ompl REQUIRED)
find_package(srdfdom REQUIRED)
find_package(urdf REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# ---- Reduced-space planning (second approach) -----------------------------
# Plans the wrist centre over 3-4 joints instead of 7, using the swept wrist
# volume from tools/wrist_blob_gen.py as a stand-in for everything distal of
# the wrist centre. See include/my_robot_control/reduced_planner.hpp.
add_library(reduced_planning
  src/srs_kinematics.cpp
  src/reduced_planner.cpp
  src/path_lifter.cpp
  src/manifold_planner.cpp)
target_include_directories(reduced_planning PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OMPL_INCLUDE_DIRS})
target_link_libraries(reduced_planning Eigen3::Eigen ${OMPL_LIBRARIES})
ament_target_dependencies(reduced_planning
  rclcpp moveit_ros_planning_interface geometry_msgs srdfdom urdf geometric_shapes)

add_executable(test_manifold src/test_manifold.cpp)
target_link_libraries(test_manifold reduced_planning Eigen3::Eigen)
ament_target_dependencies(test_manifold
  rclcpp moveit_ros_planning_interface geometry_msgs srdfdom urdf geometric_shapes)

add_executable(preview_reduced src/preview_reduced.cpp)
target_link_libraries(preview_reduced reduced_planning Eigen3::Eigen)
ament_target_dependencies(preview_reduced
  rclcpp moveit_ros_planning_interface geometry_msgs visualization_msgs
  sensor_msgs srdfdom urdf geometric_shapes)

add_executable(benchmark_reduced src/benchmark_reduced.cpp)
target_link_libraries(benchmark_reduced reduced_planning Eigen3::Eigen)
ament_target_dependencies(benchmark_reduced
  rclcpp moveit_ros_planning_interface geometry_msgs tf2_ros tf2_geometry_msgs
  srdfdom urdf geometric_shapes)

add_executable(test_reduced src/test_reduced.cpp)
target_link_libraries(test_reduced reduced_planning Eigen3::Eigen)
ament_target_dependencies(test_reduced
  rclcpp moveit_ros_planning_interface geometry_msgs srdfdom urdf geometric_shapes)

# Δημιουργία του εκτελέσιμου
add_executable(simple_move src/simple_move.cpp)
target_link_libraries(simple_move reduced_planning Eigen3::Eigen)
ament_target_dependencies(simple_move rclcpp moveit_ros_planning_interface geometry_msgs tf2_ros tf2_geometry_msgs moveit_task_constructor_core)

# Εγκατάσταση στο σύστημα του ROS
install(TARGETS
  simple_move
  test_reduced
  benchmark_reduced
  preview_reduced
  test_manifold
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS reduced_planning
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY generated DESTINATION share/${PROJECT_NAME})
install(DIRECTORY tools DESTINATION share/${PROJECT_NAME})
  
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
