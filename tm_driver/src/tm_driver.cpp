#include "tm_driver/tm_command.h"
#include "tm_driver/tm_driver.h"
#include "tm_driver/tm_print.h"

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <chrono>
#define _USE_MATH_DEFINES
#include <cmath>
#include <condition_variable>
#include <string>
#include <thread>
#include <vector>

// no thread
TmDriver::TmDriver(const std::string& ip) : svr{ip, 4096}, sct{ip, 2048, is_on_listen_node_}, state{svr.state}
{
}

// has thread
TmDriver::TmDriver(const std::string& ip, std::condition_variable* psvr_cv, std::condition_variable* psct_cv)
  : svr{ip, 4096, psvr_cv}, sct{ip, 2048, is_on_listen_node_, psct_cv}, state{svr.state}
{
  if (psvr_cv)
  {
    svr_cv_ = psvr_cv;
  }
  if (psct_cv)
  {
    sct_cv_ = psct_cv;
  }
}

bool TmDriver::is_positions_match(const TmPvtPoint& point, double eps) const
{
  const auto q_act = state.joint_angle();

  if (q_act.size() != point.positions.size())
    return false;

  for (size_t i = 0; i < point.positions.size(); ++i)
  {
    if (std::fabs(point.positions[i] - q_act[i]) > eps)
      return false;
  }
  return true;
}

bool TmDriver::start(int timeout_ms, bool stick_play)
{
  halt();
  print_info("TM_DRV: start");
  // connect to server
  bool rb = svr.start_tm_svr(timeout_ms);
  if (!rb)
    return rb;
  // send command to run project

  if (stick_play)
  {
    // svr.send_stick_play();
  }

  // connect to listen node
  rb = sct.start_tm_sct(timeout_ms);
  return rb;
}

void TmDriver::halt()
{
  print_info("TM_DRV: halt");

  stop_pvt_traj();

  if (sct.is_connected())
  {
    // send command to stop project
    // sct.send_script_exit();
  }
  sct.halt();
  if (svr.is_connected())
  {
    // send command to stop project
  }
  svr.halt();
}

bool TmDriver::get_connect_recovery_guide()
{
  return connect_recovery_is_halt_;
}

void TmDriver::set_connect_recovery_guide(bool is_halt)
{
  this->connect_recovery_is_halt_ = is_halt;
}

////////////////////////////////
// SVR Robot Function (write_XXX)
////////////////////////////////

////////////////////////////////
// SCT Robot Function (set_XXX)
////////////////////////////////
void TmDriver::back_to_listen_node()
{
  is_on_listen_node_ = true;
}

bool TmDriver::script_exit(const std::string& id)
{
  return (sct.send_script_str(id, tm_command::script_exit()) == RC_OK);
}

bool TmDriver::set_tag(int tag, int wait, const std::string& id)
{
  return (sct.send_script_str(id, tm_command::set_tag(tag, wait)) == RC_OK);
}

bool TmDriver::set_wait_tag(int tag, int timeout_ms, const std::string& id)
{
  return (sct.send_script_str(id, tm_command::set_wait_tag(tag, timeout_ms)) == RC_OK);
}

bool TmDriver::set_stop(const std::string& id)
{
  return (sct.send_script_str(id, tm_command::set_stop()) == RC_OK);
}

bool TmDriver::set_pause(const std::string& id)
{
  return (sct.send_script_str(id, tm_command::set_pause()) == RC_OK);
}

bool TmDriver::set_resume(const std::string& id)
{
  return (sct.send_script_str(id, tm_command::set_resume()) == RC_OK);
}

bool TmDriver::set_io(TmIOModule module, TmIOType type, int pin, float state, const std::string& id)
{
  return (sct.send_script_str(id, tm_command::set_io(module, type, pin, state)) == RC_OK);
}

bool TmDriver::set_joint_pos_PTP(const std::array<double, 6>& angs, double vel, double acc_time, int blend_percent,
                                 bool fine_goal, const std::string& id)
{
  int vel_pa = int(100.0 * (vel / max_velocity_));
  if (vel_pa >= 100)
    vel_pa = 100;  // max 100%
  return (sct.send_script_str(id, tm_command::set_joint_pos_PTP(angs, vel_pa, acc_time, blend_percent, fine_goal)) ==
          RC_OK);
}

bool TmDriver::set_tool_pose_PTP(const std::array<double, 6>& pose, double vel, double acc_time, int blend_percent,
                                 bool fine_goal, const std::string& id)
{
  const int vel_pa = int(100.0 * (vel / max_velocity_));
  return (sct.send_script_str(id, tm_command::set_tool_pose_PTP(pose, vel_pa, acc_time, blend_percent, fine_goal)) ==
          RC_OK);
}

bool TmDriver::set_tool_pose_Line(const std::array<double, 6>& pose, double vel, double acc_time, int blend_percent,
                                  bool fine_goal, const std::string& id)
{
  return (sct.send_script_str(id, tm_command::set_tool_pose_Line(pose, vel, acc_time, blend_percent, fine_goal)) ==
          RC_OK);
}

bool TmDriver::set_pvt_enter(TmPvtMode mode, const std::string& id)
{
  return (sct.send_script_str(id, tm_command::set_pvt_enter(int(mode))) == RC_OK);
}

bool TmDriver::set_pvt_exit(const std::string& id)
{
  return (sct.send_script_str(id, tm_command::set_pvt_exit()) == RC_OK);
}

bool TmDriver::set_pvt_point(TmPvtMode mode, double t, const std::array<double, 6>& pos,
                             const std::array<double, 6>& vel, const std::string& id)
{
  if (t < 0.0 || pos.size() != vel.size() || pos.size() < 6)
    return false;

  return (sct.send_script_str(id, tm_command::set_pvt_point(mode, t, pos, vel)) == RC_OK);
}

bool TmDriver::set_pvt_point(TmPvtMode mode, const TmPvtPoint& point, const std::string& id)
{
  return (sct.send_script_str(id, tm_command::set_pvt_point(mode, point)) == RC_OK);
}

bool TmDriver::set_pvt_traj(const TmPvtTraj& pvts, const std::string& id)
{
  const std::string script = tm_command::set_pvt_traj(pvts);
  print_debug("TM_DRV: send script (pvt traj.):");
  print_debug("%s\n", script.c_str());

  std::string script_copy = script;
  static const std::string DELIMITER = "\n";
  size_t pos = 0;
  std::string token;
  while ((pos = script_copy.find(DELIMITER)) != std::string::npos)
  {
    token = script_copy.substr(0, pos);
    print_debug(token.c_str());
    script_copy.erase(0, pos + DELIMITER.length());
  }

  return (sct.send_script_str(id, script) == RC_OK);
}

bool TmDriver::run_pvt_traj(const TmPvtTraj& pvts, double goal_duration_scaling, double goal_duration_margin)
{
  print_info("pvts.total_time:= %.3f", pvts.total_time);
  print_info("goal_duration_scaling:= %.3f", goal_duration_scaling);
  print_info("goal_duration_margin:= %.3f", goal_duration_margin);
  const double total_time = pvts.total_time * goal_duration_scaling + goal_duration_margin;
  print_info("Total allowed time (%.3f * %.3f + %.3f):= %.3f", pvts.total_time, goal_duration_scaling,
             goal_duration_margin, total_time);
  auto time_start = std::chrono::steady_clock::now();
  auto time_now = time_start;

  if (pvts.points.empty())
  {
    print_debug("TM_DRV: empty traj.");
    return false;
  }

  if (!sct.is_connected())
  {
    print_debug("TM_DRV: SCT not connected.");
    return false;
  }

  if (!set_pvt_traj(pvts))
  {
    return false;
  }

  start_pvt_traj();

  // wait
  double time = 0.0;
  double max_speed = 0.0;
  while (is_exec_pvt_traj() && time < total_time)
  {
    max_speed = std::max<double>(max_speed, state.tcp_speed());

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    time_now = std::chrono::steady_clock::now();
    time = std::chrono::duration_cast<std::chrono::duration<double>>(time_now - time_start).count();
    if (!state.is_project_running())
    {
      stop_pvt_traj();
      print_error("TM_DRV: proj. stopped after %.3f", time);
      break;
    }
    if (state.has_error())
    {
      stop_pvt_traj();
      print_error("TM_DRV: proj. error after %.3f", time);
      break;
    }
    if (is_positions_match(pvts.points.back(), 0.001))
    {
      print_info("TM_DRV: traj. done after %.3f", time);
      break;
    }
  }

  print_info("TM_DRV: max speed: %.3f", max_speed);

  // When nothing stopped the robot yet, it was succesfull
  const bool succesfull = is_exec_pvt_traj();

  if (is_exec_pvt_traj())
  {
    stop_pvt_traj();
  }
  else
  {
    set_stop();
  }
  print_info("pvts.total_time:= %.3f", pvts.total_time);
  print_info("goal_duration_scaling:= %.3f", goal_duration_scaling);
  print_info("goal_duration_margin:= %.3f", goal_duration_margin);
  print_info("Total allowed time (%.3f * %.3f + %.3f):= %.3f", pvts.total_time, goal_duration_scaling,
             goal_duration_margin, total_time);
  print_info("TM_DRV: traj. exec. time:= %.3f", time);
  print_info("Execution ratio:= %.3f", time / pvts.total_time);
  return succesfull;
}

void TmDriver::cubic_interp(TmPvtPoint& p, const TmPvtPoint& p0, const TmPvtPoint& p1, double t)
{
  double c, d;
  const double t_end = p1.time;

  if (t < 0.0)
  {
    t = 0.0;
  }
  else if (t > t_end)
  {
    t = t_end;
  }

  p.time = t;

  for (size_t i = 0; i < p.positions.size(); ++i)
  {
    c = ((3.0 * (p1.positions[i] - p0.positions[i]) / t_end) - 2.0 * p0.velocities[i] - p1.velocities[i]) / t_end;
    d = ((2.0 * (p0.positions[i] - p1.positions[i]) / t_end) + p0.velocities[i] + p1.velocities[i]) / (t_end * t_end);
    p.positions[i] = p0.positions[i] + p0.velocities[i] * t + c * t * t + d * t * t * t;
    p.velocities[i] = p0.velocities[i] + 2.0 * c * t + 3.0 * d * t * t;
  }
}

bool TmDriver::fake_run_pvt_traj(const TmPvtTraj& pvts)
{
  auto time_init = std::chrono::steady_clock::now();
  auto time_start = time_init;
  auto time_now = time_init;

  if (pvts.mode != TmPvtMode::Joint || pvts.points.size() < 2)
    return false;

  start_pvt_traj();

  TmPvtPoint p_start;
  p_start.time = 0.0;
  p_start.positions = state.joint_angle();
  p_start.velocities = state.mtx_joint_speed();
  TmPvtPoint& p0 = p_start;
  TmPvtPoint point = p_start;
  const std::array<double, 6> zeros{};
  size_t idx = 0;

  // first point
  // print_info(tm_command::set_pvt_point(pvts.mode, p0));
  // print_info(tm_command::set_pvt_point(pvts.mode, pvts.points[idx]));
  print_info(tm_command::set_pvt_point(pvts.mode, p0).c_str());
  print_info(tm_command::set_pvt_point(pvts.mode, pvts.points[idx]).c_str());
  point.time = pvts.points[0].time;

  while (is_exec_pvt_traj())
  {
    cubic_interp(point, p0, pvts.points[idx], point.time);
    state.mtx_set_joint_states(point.positions, point.velocities, zeros);

    std::this_thread::sleep_for(std::chrono::milliseconds(4));

    time_now = std::chrono::steady_clock::now();
    point.time = std::chrono::duration_cast<std::chrono::duration<double>>(time_now - time_start).count();
    if (point.time > pvts.points[idx].time)
    {
      p0 = pvts.points[idx];
      point.time -= pvts.points[idx].time;
      time_start = time_now;
      ++idx;
      if (idx == pvts.points.size())
        break;

      print_info(tm_command::set_pvt_point(pvts.mode, pvts.points[idx]).c_str());
    }
  }
  // last point
  if (is_exec_pvt_traj())
  {
    idx = pvts.points.size() - 1;
    cubic_interp(point, pvts.points[idx - 1], pvts.points[idx], pvts.points[idx].time);
  }
  state.mtx_set_joint_states(point.positions, zeros, zeros);

  time_now = std::chrono::steady_clock::now();
  point.time = std::chrono::duration_cast<std::chrono::duration<double>>(time_now - time_init).count();
  print_info("TM_DRV: traj. exec. time:= %f", point.time);

  stop_pvt_traj();
  return true;
}

bool TmDriver::set_vel_mode_start(VelMode mode, double timeout_zero_vel, double timeout_stop, const std::string& id)
{
  return (sct.send_script_str(id, tm_command::set_vel_mode_start(mode, timeout_zero_vel, timeout_stop)) == RC_OK);
}

bool TmDriver::set_vel_mode_stop(const std::string& id)
{
  return (sct.send_script_str(id, tm_command::set_vel_mode_stop()) == RC_OK);
}

bool TmDriver::set_vel_mode_target(VelMode mode, const std::array<double, 6>& vel, const std::string& id)
{
  return (sct.send_script_str_silent(id, tm_command::set_vel_mode_target(mode, vel)) == RC_OK);
}
