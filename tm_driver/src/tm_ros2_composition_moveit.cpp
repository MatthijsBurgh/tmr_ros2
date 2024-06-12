#include "tm_driver/tm_driver.h"
#include "tm_driver/tm_print.h"
#include "tm_driver/tm_ros2_svr.h"
#include "tm_driver/tm_ros2_moveit_sct.h"

#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>

#include <cstdio>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

void debug_function_print(const std::string& msg)
{
  std::cerr << PRINT_CYAN << "[TM_DEBUG] " << msg << PRINT_RESET << "\n";
}

void info_function_print(const std::string& msg)
{
  std::cout << "[TM_INFO] " << msg << "\n";
}

void warn_function_print(const std::string& msg)
{
  std::cerr << PRINT_YELLOW << "[TM_WARN] " << msg << PRINT_RESET << "\n";
}

void error_function_print(const std::string& msg)
{
  std::cerr << PRINT_RED << "[TM_ERROR] " << msg << PRINT_RESET << "\n";
}

void fatal_function_print(const std::string& msg)
{
  std::cerr << PRINT_GREEN << "[TM_FATAL] " << msg << PRINT_RESET << "\n";
}

void ros_debug_print(const std::string& msg)
{
  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("tm_driver"), msg);
}

void ros_info_print(const std::string& msg)
{
  RCLCPP_INFO_STREAM(rclcpp::get_logger("tm_driver"), msg);
}

void ros_warn_function_print(const std::string& msg)
{
  RCLCPP_WARN_STREAM(rclcpp::get_logger("tm_driver"), msg);
}

void ros_error_print(const std::string& msg)
{
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("tm_driver"), msg);
}

void ros_fatal_print(const std::string& msg)
{
  RCLCPP_ERROR_STREAM(rclcpp::get_logger("tm_driver"), "[TM_FATAL] " << msg);
}

void ros_once_print(const std::string& msg)
{
  RCLCPP_INFO_STREAM_ONCE(rclcpp::get_logger("tm_driver"), msg);
}

void set_up_print_fuction()
{
  set_up_print_debug_function(debug_function_print);
  set_up_print_info_function(info_function_print);
  set_up_print_warn_function(warn_function_print);
  set_up_print_error_function(error_function_print);
  set_up_print_fatal_function(fatal_function_print);
  set_up_print_once_function(default_print_once_function_print);
}

void set_up_ros_print_fuction()
{
  set_up_print_debug_function(ros_debug_print);
  set_up_print_info_function(ros_info_print);
  set_up_print_warn_function(ros_warn_function_print);
  set_up_print_error_function(ros_error_print);
  set_up_print_fatal_function(ros_fatal_print);
  set_up_print_once_function(ros_once_print);
}

int main(int argc, char* argv[])
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

  set_up_ros_print_fuction();

  std::vector<std::string> args = rclcpp::init_and_remove_ros_arguments(argc, argv);

  if (args.size() < 2)
  {
    ros_fatal_print("Please provide the 'robot_ip_address'");
    rclcpp::shutdown();
    return 1;
  }

  bool is_fake = true;
  std::string host;
  if (args.size() > 1)
  {
    host = args[1];
    if (host.find("robot_ip:=") != std::string::npos)
    {
      host.replace(host.begin(), host.begin() + 10, "");
      is_fake = false;
    }
    else if (host.find("ip:=") != std::string::npos)
    {
      host.replace(host.begin(), host.begin() + 4, "");
      is_fake = false;
    }
    else
    {
      print_info("ip is not found, using a fake robot");
    }
  }
  else
  {
    print_fatal("Please provide the 'robot_ip_address'");
    rclcpp::shutdown();
    return 1;
  }

  if (is_fake)
  {
    std::stringstream ss;
    ss << "Only ip or robot_ip support, but your type is '" << host << "', using a fake robot";
    print_info(ss.str().c_str());
  }

  if (args.size() == 3)
  {
    bool is_set_no_log_print;
    std::istringstream(args[2]) >> std::boolalpha >> is_set_no_log_print;
    if (is_set_no_log_print)
    {
      set_up_print_fuction();
    }
  }

  TmDriver iface(host, nullptr, nullptr);

  const rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("tm_driver_node");

  auto tm_svr = std::make_shared<TmSvrRos2>(node, iface, is_fake);
  auto tm_sct = std::make_shared<TmRos2SctMoveit>(node, iface, is_fake);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
