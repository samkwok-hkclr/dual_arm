cmake_minimum_required(VERSION 3.22)
project(dual_arm_configure)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY 
  config 
  launch  
  DESTINATION share/${PROJECT_NAME})
