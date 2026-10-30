cmake_minimum_required(VERSION 3.10)
project(serabot_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
