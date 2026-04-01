cmake_minimum_required(VERSION 3.10.2)
project(tm_description)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install the Python package
ament_python_install_package(${PROJECT_NAME})

# Install Python scripts as executables
install(PROGRAMS
  tm_description/modify_xacro.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config meshes xacro
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
