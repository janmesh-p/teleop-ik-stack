cmake_minimum_required(VERSION 3.16)
project(teleop_ik)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -Wno-maybe-uninitialized)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(teleop_ik_core src/franka_kinematics.cpp src/dls_ik.cpp)
target_include_directories(teleop_ik_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(teleop_ik_core Eigen3::Eigen)

add_executable(fk_check src/nodes/fk_check.cpp)
target_link_libraries(fk_check teleop_ik_core)
ament_target_dependencies(fk_check rclcpp sensor_msgs tf2_msgs)

add_executable(teleop_node src/nodes/teleop_node.cpp)
target_link_libraries(teleop_node teleop_ik_core)
ament_target_dependencies(teleop_node rclcpp sensor_msgs geometry_msgs std_msgs)

install(TARGETS teleop_ik_core fk_check teleop_node
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ik test/test_ik.cpp)
  target_link_libraries(test_ik teleop_ik_core)
endif()

ament_package()
