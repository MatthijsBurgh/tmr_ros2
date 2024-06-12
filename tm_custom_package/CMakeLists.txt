cmake_minimum_required(VERSION 3.5)
project(tm_custom_package)

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
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tm_robot_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

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

add_executable(tm_send_command src/send_command.cpp)
ament_target_dependencies(tm_send_command tm_robot_msgs rclcpp)

add_executable(sub_img src/sub_img.cpp)
ament_target_dependencies(sub_img rclcpp sensor_msgs std_msgs OpenCV ament_index_cpp)

install(TARGETS tm_send_command sub_img DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY image DESTINATION share/${PROJECT_NAME})
ament_package()
