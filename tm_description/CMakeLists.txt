cmake_minimum_required(VERSION 3.10.2)
project(tm_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config meshes xacro
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
