cmake_minimum_required(VERSION 3.5)
project(tm_demo)

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
# uncomment the following section in order to fill in further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tm_msgs REQUIRED)

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

include_directories(include ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(demo_send_script src/demo_send_script.cpp)
ament_target_dependencies(demo_send_script rclcpp std_msgs sensor_msgs tm_msgs)

install(TARGETS demo_send_script DESTINATION lib/${PROJECT_NAME})

add_executable(demo_ask_sta src/demo_ask_sta.cpp)
ament_target_dependencies(demo_ask_sta rclcpp std_msgs sensor_msgs tm_msgs)

install(TARGETS demo_ask_sta DESTINATION lib/${PROJECT_NAME})

add_executable(demo_connect_tm src/demo_connect_tm.cpp)
ament_target_dependencies(demo_connect_tm rclcpp std_msgs sensor_msgs tm_msgs)

install(TARGETS demo_connect_tm DESTINATION lib/${PROJECT_NAME})

add_executable(demo_set_event src/demo_set_event.cpp)
ament_target_dependencies(demo_set_event rclcpp std_msgs sensor_msgs tm_msgs)

install(TARGETS demo_set_event DESTINATION lib/${PROJECT_NAME})
add_executable(demo_set_io src/demo_set_io.cpp)
ament_target_dependencies(demo_set_io rclcpp std_msgs sensor_msgs tm_msgs)

install(TARGETS demo_set_io DESTINATION lib/${PROJECT_NAME})
add_executable(
  demo_set_positions # robot does'nt move
  src/demo_set_positions.cpp
)
ament_target_dependencies(demo_set_positions rclcpp std_msgs sensor_msgs tm_msgs)

install(TARGETS demo_set_positions DESTINATION lib/${PROJECT_NAME})
add_executable(demo_write_item src/demo_write_item.cpp)
ament_target_dependencies(demo_write_item rclcpp std_msgs sensor_msgs tm_msgs)

install(TARGETS demo_write_item DESTINATION lib/${PROJECT_NAME})
add_executable(demo_ask_item src/demo_ask_item.cpp)
ament_target_dependencies(demo_ask_item rclcpp std_msgs sensor_msgs tm_msgs)

install(TARGETS demo_ask_item DESTINATION lib/${PROJECT_NAME})
add_executable(demo_get_feedback get_status_demo_src/demo_get_feedback.cpp)
ament_target_dependencies(demo_get_feedback rclcpp std_msgs sensor_msgs tm_msgs)

install(TARGETS demo_get_feedback DESTINATION lib/${PROJECT_NAME})
add_executable(demo_get_torque_feedback get_status_demo_src/demo_get_torque_feedback.cpp)
ament_target_dependencies(demo_get_torque_feedback rclcpp std_msgs sensor_msgs tm_msgs)

install(TARGETS demo_get_torque_feedback DESTINATION lib/${PROJECT_NAME})
add_executable(demo_get_sct_response get_status_demo_src/demo_get_sct_response.cpp)
ament_target_dependencies(demo_get_sct_response rclcpp std_msgs sensor_msgs tm_msgs)

install(TARGETS demo_get_sct_response DESTINATION lib/${PROJECT_NAME})
add_executable(demo_get_sta_response get_status_demo_src/demo_get_sta_response.cpp)
ament_target_dependencies(demo_get_sta_response rclcpp std_msgs sensor_msgs tm_msgs)

install(TARGETS demo_get_sta_response DESTINATION lib/${PROJECT_NAME})

add_executable(demo_get_svr_response get_status_demo_src/demo_get_svr_response.cpp)
ament_target_dependencies(demo_get_svr_response rclcpp std_msgs sensor_msgs tm_msgs)

install(TARGETS demo_get_svr_response DESTINATION lib/${PROJECT_NAME})
add_executable(demo_leave_listen_node src/demo_leave_listen_node.cpp)
ament_target_dependencies(demo_leave_listen_node rclcpp std_msgs sensor_msgs tm_msgs)

install(TARGETS demo_leave_listen_node DESTINATION lib/${PROJECT_NAME})

ament_package()
