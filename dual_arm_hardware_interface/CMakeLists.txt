cmake_minimum_required(VERSION 3.8)
project(dual_arm_hardware_interface)

set(CMAKE_CXX_VISIBILITY_PRESET "default")
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-dev)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(diagnostic_updater REQUIRED)

find_package(ros2_socketcan_msgs REQUIRED)
find_package(ros2_control REQUIRED) 

file(GLOB_RECURSE PLUGINS_SRC src/*.cpp)
add_library(${PROJECT_NAME} SHARED 
  src/dual_arm_hardware_interface.cpp)
target_compile_options(${PROJECT_NAME} PRIVATE -fvisibility=default)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle

  hardware_interface
  pluginlib
  diagnostic_updater

  ros2_socketcan_msgs
  ros2_control
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  OUTPUT_NAME "dual_arm_hardware_interface"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/install/${PROJECT_NAME}/lib"
  ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/install/${PROJECT_NAME}/lib"
)

pluginlib_export_plugin_description_file(hardware_interface dual_arm_hardware_interface_plugin.xml)

# 安装配置
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY
  DESTINATION share/${PROJECT_NAME}/
)

# 导出
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ros2_socketcan_msgs
  ros2_control
)

ament_package()