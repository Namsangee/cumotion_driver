cmake_minimum_required(VERSION 3.8)
project(dsr_isaac_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(pid_isaacsim_node
  src/pid_isaacsim_node.cpp
)

ament_target_dependencies(pid_isaacsim_node
  rclcpp
  sensor_msgs
)

target_include_directories(pid_isaacsim_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(pid_isaacsim_node PUBLIC cxx_std_17)

install(TARGETS pid_isaacsim_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/isaac_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)
ament_package()
