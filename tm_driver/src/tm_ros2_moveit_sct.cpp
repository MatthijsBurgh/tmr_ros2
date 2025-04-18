#include "tm_driver/tm_command.h"
#include "tm_driver/tm_ros2_moveit_sct.h"
#include "tm_driver/tm_print.h"

#include <builtin_interfaces/msg/duration.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/utilities.hpp>
#include <rclcpp_action/create_server.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp_action/server_goal_handle.hpp>
#include <rclcpp_action/types.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

auto sec = [](const builtin_interfaces::msg::Duration& t) {
  return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
};

void TmRos2SctMoveit::intial_action()
{
  as_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
      node->get_node_base_interface(), node->get_node_clock_interface(), node->get_node_logging_interface(),
      node->get_node_waitables_interface(), "tmr_arm_controller/follow_joint_trajectory",
      std::bind(&TmRos2SctMoveit::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&TmRos2SctMoveit::handle_cancel, this, std::placeholders::_1),
      std::bind(&TmRos2SctMoveit::handle_accepted, this, std::placeholders::_1));
  clear_goal();
}

rclcpp_action::GoalResponse
TmRos2SctMoveit::handle_goal(const rclcpp_action::GoalUUID& uuid,
                             std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal>
                                 goal)  // NOLINT(performance-unnecessary-value-param)
{
  const std::string goal_id = rclcpp_action::to_string(uuid);
  print_info("Received new action goal %s", goal_id.c_str());

  if (has_goal())
  {
    return rclcpp_action::GoalResponse::ACCEPT_AND_DEFER;
  }

  if (!is_fake_)
  {
    if (!svr_.is_connected())
    {
      print_error("Goal: %s, got rejected because of SVR not being connected", goal_id.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!sct_.is_connected())
    {
      print_error("Goal: %s, got rejected because of SCT not being connected", goal_id.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (state_.has_error())
    {
      print_error("Goal: %s, got rejected because STATE has an error", goal_id.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  if (!has_points(goal->trajectory))
  {
    std::string msg = control_msgs::action::to_yaml(*goal);
    const std::string delimiter = "\n";
    size_t pos = 0;
    std::string token;
    while ((pos = msg.find(delimiter)) != std::string::npos)
    {
      token = msg.substr(0, pos);
      print_error(token.c_str());
      msg.erase(0, pos + delimiter.length());
    }

    print_error("Goal: %s, got rejected because of no trajectory points", goal_id.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  print_info("Goal: %s, got accepted", goal_id.c_str());
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TmRos2SctMoveit::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>>
        goal_handle)  // NOLINT(performance-unnecessary-value-param)
{
  auto goal_id = rclcpp_action::to_string(goal_handle->get_goal_id());
  print_info("Got request to cancel goal %s", goal_id.c_str());
  {
    const std::lock_guard<std::mutex> lck(as_mtx_);
    if (has_goal() && goal_id_.compare(goal_id) == 0)
    {
      clear_goal();
      iface_.stop_pvt_traj();
    }
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void TmRos2SctMoveit::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>>
        goal_handle)  // NOLINT(performance-unnecessary-value-param)
{
  {
    const std::unique_lock<std::mutex> lck(as_mtx_);
    set_goal(goal_handle->get_goal_id());
  }
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&TmRos2SctMoveit::execute_traj, this, std::placeholders::_1), goal_handle}.detach();
}

void TmRos2SctMoveit::execute_traj(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>>& goal_handle)
{
  print_debug("TM_ROS: trajectory thread begin");

  const auto result = std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();

  // actually, no need to reorder
  // std::vector<trajectory_msgs::msg::JointTrajectoryPoint> traj_points;
  // reorder_traj_joints(traj_points, goal_handle->get_goal()->trajectory);
  const auto& traj_points = goal_handle->get_goal()->trajectory.points;

  if (!is_positions_match(traj_points.front(), 0.01))
  {
    result->error_code = result->PATH_TOLERANCE_VIOLATED;
    // result->error_string = "Start point doesn't match current pose";
    std::stringstream ss;
    ss << "Start point doesn't match current pose: \n";
    auto q_act = state_.joint_angle();
    auto& q_start = traj_points.front();
    for (size_t i = 0; i < q_start.positions.size(); ++i)
    {
      ss << q_act[i] << " != " << q_start.positions[i] << " (" << q_start.positions[i] - q_act[i] << ")\n";
    }
    result->error_string = ss.str();
    print_warn(result->error_string.c_str());
    goal_handle->abort(result);
  }

  auto pvts = get_pvt_traj(traj_points, 0.025);
  print_info("TM_ROS: traj. total time:= %d", static_cast<int>(pvts->total_time));

  if (!goal_handle->is_executing())
  {
    print_debug("goal_handle->execute()");
    goal_handle->execute();
  }

  if (!is_fake_)
  {
    iface_.run_pvt_traj(
        *pvts, ((100.0 / state_.project_speed()) * 0.95 + 0.55),
        0.25);  // Total time running is (0.95 * X + 0.55) * total_time, with X 100/project_speed; with additional 0.25s margin
  }
  else
  {
    iface_.fake_run_pvt_traj(*pvts);
  }

  if (rclcpp::ok())
  {
    if (!is_positions_match(traj_points.back(), 0.01))
    {
      result->error_code = result->GOAL_TOLERANCE_VIOLATED;
      // result->error_string = "Current pose doesn't match Goal point";;
      std::stringstream ss;
      ss << "Current pose doesn't match goal point: \n";
      const auto q_act = state_.joint_angle();
      const auto& q_end = traj_points[traj_points.size() - 1];
      for (size_t i = 0; i < q_end.positions.size(); ++i)
      {
        ss << q_act[i] << " != " << q_end.positions[i] << " (" << q_end.positions[i] - q_act[i] << ")\n";
      }
      result->error_string = ss.str();
      print_warn(result->error_string.c_str());
      goal_handle->abort(result);
    }
    else
    {
      result->error_code = result->SUCCESSFUL;
      result->error_string = "Goal reached succesfully!";
      print_info(result->error_string.c_str());
      goal_handle->succeed(result);
    }
  }

  {
    const std::lock_guard<std::mutex> lck(as_mtx_);
    clear_goal();
  }
  print_debug("TM_ROS: trajectory thread end");
}

void TmRos2SctMoveit::reorder_traj_joints(std::vector<trajectory_msgs::msg::JointTrajectoryPoint>& new_traj_points,
                                          const trajectory_msgs::msg::JointTrajectory& traj)
{
  /* Reorders trajectory - destructive */
  std::vector<size_t> mapping;
  mapping.resize(jns_.size(), jns_.size());
  for (size_t i = 0; i < traj.joint_names.size(); ++i)
  {
    for (size_t j = 0; j < jns_.size(); ++j)
    {
      if (traj.joint_names[i] == jns_[j])
        mapping[j] = i;
    }
  }
  new_traj_points.clear();
  for (const auto& point : traj.points)
  {
    trajectory_msgs::msg::JointTrajectoryPoint& new_point = new_traj_points.emplace_back();
    for (unsigned int j = 0; j < point.positions.size(); ++j)
    {
      new_point.positions.push_back(point.positions[mapping[j]]);
      new_point.velocities.push_back(point.velocities[mapping[j]]);
      if (!point.accelerations.empty())
        new_point.accelerations.push_back(point.accelerations[mapping[j]]);
    }
    new_point.time_from_start = point.time_from_start;
  }
}

bool TmRos2SctMoveit::has_points(const trajectory_msgs::msg::JointTrajectory& traj)
{
  if (traj.points.empty())
    return false;

  for (auto& point : traj.points)
  {
    if (point.positions.size() != traj.joint_names.size() || point.velocities.size() != traj.joint_names.size())
      return false;
  }
  return true;
}

bool TmRos2SctMoveit::is_traj_finite(const trajectory_msgs::msg::JointTrajectory& traj)
{
  for (auto& point : traj.points)
  {
    for (const auto& p : point.positions)
    {
      if (!std::isfinite(p))
        return false;
    }
    for (const auto& v : point.velocities)
    {
      if (!std::isfinite(v))
        return false;
    }
  }
  return true;
}

bool TmRos2SctMoveit::is_positions_match(const trajectory_msgs::msg::JointTrajectoryPoint& point, double eps)
{
  const auto q_act = state_.joint_angle();

  if (q_act.size() != point.positions.size())
    return false;

  for (size_t i = 0; i < point.positions.size(); ++i)
  {
    if (std::fabs(point.positions[i] - q_act[i]) > eps)
      return false;
  }
  return true;
}

void TmRos2SctMoveit::set_pvt_traj(TmPvtTraj& pvts,
                                   const std::vector<trajectory_msgs::msg::JointTrajectoryPoint>& traj_points,
                                   double Tmin)
{
  print_debug("TM_ROS: Traj size: %d", static_cast<int>(traj_points.size()));
  if (traj_points.size() < 2)
  {
    print_warn("TM_ROS: Traj rejected because size<2: %d", static_cast<int>(traj_points.size()));
    return;
  }

  const double total_time = sec(traj_points[traj_points.size() - 1].time_from_start);
  if (total_time < Tmin)
  {
    print_warn("TM_ROS: Traj rejected because total time<Tmin(%f): %f", Tmin, total_time);
    return;
  }

  pvts.mode = TmPvtMode::Joint;

  // Skip the first point as it is the current position and is not accepted by the robot
  size_t i = 1, i_1 = 1, i_2 = 1;
  int skip_count = 0;
  TmPvtPoint point;

  for (i = 1; i < traj_points.size() - 1; ++i)
  {
    point.time = sec(traj_points[i].time_from_start) - sec(traj_points[i_1].time_from_start);
    if (point.time >= Tmin)
    {
      i_2 = i_1;
      i_1 = i;
      // Assuming the traj_point positions and velocities are of length 6
      std::copy(traj_points[i].positions.begin(), traj_points[i].positions.end(), point.positions.begin());
      std::copy(traj_points[i].velocities.begin(), traj_points[i].velocities.end(), point.velocities.begin());
      pvts.points.push_back(point);
    }
    else
    {
      print_debug("Skipping index: %d", i);
      ++skip_count;
    }
  }
  if (skip_count > 0)
  {
    print_warn("TM_ROS: Traj.: skip %d points", (int)skip_count);
  }
  // last point
  if (traj_points.size() > 1)
  {
    i = traj_points.size() - 1;
    // Assuming the traj_point positions and velocities are of length 6
    point.time = sec(traj_points[i].time_from_start) - sec(traj_points[i_1].time_from_start);
    std::copy(traj_points[i].positions.begin(), traj_points[i].positions.end(), point.positions.begin());
    std::copy(traj_points[i].velocities.begin(), traj_points[i].velocities.end(), point.velocities.begin());
    if (point.time >= Tmin)
    {
      pvts.points.push_back(point);
    }
    else
    {
      point.time = sec(traj_points[i].time_from_start) - sec(traj_points[i_2].time_from_start);
      pvts.points.back() = point;  // This is safe because the starting point is always added.
      ++skip_count;
      print_warn("TM_ROS: Traj.: skip 1 more last point");
    }
  }
  pvts.total_time = sec(traj_points.back().time_from_start);
}

std::shared_ptr<TmPvtTraj>
TmRos2SctMoveit::get_pvt_traj(const std::vector<trajectory_msgs::msg::JointTrajectoryPoint>& traj_points, double Tmin)
{
  std::shared_ptr<TmPvtTraj> pvts = std::make_shared<TmPvtTraj>();
  set_pvt_traj(*pvts, traj_points, Tmin);
  return pvts;
}
