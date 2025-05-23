cmake_minimum_required(VERSION 3.5)
project(tm_driver)

# Default to C17
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 17)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Werror=all)
  add_compile_options(-Wextra -Werror=extra)
  add_compile_options(-Wpedantic -Werror=pedantic)
endif()

# Please refer to https://moveit.ros.org/install-moveit2/source If the MoveIt2 module is not ready for installation, or
# the user wants to close the relative MoveIt2 module, the user can modify the "TRUE" of 'TM_MOVEIT_AUTO_DETECT' to
# "FALSE" to compile.
option(TM_MOVEIT_AUTO_DETECT "Try to detect moveit2" TRUE)
set(TM_DRIVER_INCLUDE_MOVEIT ${TM_MOVEIT_AUTO_DETECT})

# If the user enable this 'TM_DRIVER_MESSAGE_PRINT' to print debug messages, more information will be displayed "1
# package had stderr output: tm_driver" on the screen.
option(TM_DRIVER_MESSAGE_PRINT "More debug messages are printed" TRUE)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tm_msgs REQUIRED)

if(TM_MOVEIT_AUTO_DETECT)
  find_package(moveit_ros_planning_interface QUIET)
  if(NOT moveit_ros_planning_interface_FOUND)
    set(TM_DRIVER_INCLUDE_MOVEIT FALSE)
    if(TM_DRIVER_INCLUDE_MOVEIT)
      message("${PROJECT_NAME}:--- TMDriver build error : MoveIt2_LIB not found")
      # do you forget to source moveit2 relative setup.bash? example: & ~/COLCON_WS/install/setup.bash
    endif()
  endif()
endif()

if(TM_DRIVER_INCLUDE_MOVEIT)
  find_package(trajectory_msgs REQUIRED)
  find_package(control_msgs REQUIRED)
  find_package(rclcpp_action REQUIRED)
endif()

if(BUILD_TESTING)
  # set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
  find_package(ament_cmake_clang_format REQUIRED)
  ament_clang_format(CONFIG_FILE ../.clang-format)
  # find_package(ament_cmake_clang_tidy REQUIRED) ament_clang_tidy(${CMAKE_BINARY_DIR} CONFIG_FILE ../.clang-tidy)
  find_package(ament_cmake_lint_cmake REQUIRED)
  ament_lint_cmake()
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
endif()

include_directories(include)

set(DRIVER_SOURCES
    src/tm_ros2_sct.cpp
    src/tm_ros2_svr.cpp
    src/tm_driver.cpp
    src/tm_command.cpp
    src/tm_sct_communication.cpp
    src/tm_svr_communication.cpp
    src/tm_communication.cpp
    src/tm_robot_state.cpp
    src/tm_packet.cpp
    src/tm_print.cpp
    src/tm_driver_utilities.cpp
    src/tm_listen_node_connect.cpp
    src/tm_ethernet_slave_connect.cpp
)

set(DRIVER_DEPENDENCIES rclcpp std_msgs sensor_msgs tf2_geometry_msgs tm_msgs)

if(TM_DRIVER_INCLUDE_MOVEIT)
  list(APPEND DRIVER_SOURCES src/tm_ros2_composition_moveit.cpp src/tm_ros2_moveit_sct.cpp)
  list(APPEND DRIVER_DEPENDENCIES trajectory_msgs control_msgs rclcpp_action)
else()
  list(APPEND DRIVER_SOURCES src/tm_ros2_composition.cpp)
endif()

add_executable(tm_driver ${DRIVER_SOURCES})
ament_target_dependencies(tm_driver ${DRIVER_DEPENDENCIES})

install(TARGETS tm_driver DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
