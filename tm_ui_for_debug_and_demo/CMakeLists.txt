cmake_minimum_required(VERSION 3.5)
project(tm_ui_for_debug_and_demo)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tm_msgs REQUIRED)

# Qt5
find_package(Qt5Core REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(Qt5Widgets REQUIRED)

# ######################################################################################################################
# Build ##
# ######################################################################################################################
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

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Specify additional locations of header files

include_directories(
  ${rclcpp_INCLUDE_DIRS} ${rclcpp_action_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS} ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Quick_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

qt5_wrap_ui(UIC_FILES src/tm_ros_driver_windows.ui)
set(HEADERS src/tm_ros_driver_windows.hpp)

set(SRCS src/main.cpp src/ros_page.cpp src/tm_ros_driver_windows.cpp)

# Demo executable
add_executable(robot_ui ${SRCS} ${UIC_FILES} ${HEADERS})

ament_target_dependencies(robot_ui rclcpp std_msgs rclcpp_action ament_index_cpp tm_msgs)
target_link_libraries(robot_ui Qt5::Core Qt5::Quick Qt5::Widgets ${catkin_LIBRARIES})

# ######################################################################################################################
# Install ##
# ######################################################################################################################

# Install libraries
install(TARGETS robot_ui DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
