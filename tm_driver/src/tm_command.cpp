#include "tm_driver/tm_command.h"

#include <array>
#include <cstddef>
#include <iomanip>
#include <ios>
#include <sstream>
#include <string>

namespace tm_command
{

std::array<double, 6> degs(const std::array<double, 6>& angs)
{
  std::array<double, 6> rv;
  for (size_t i = 0; i < angs.size(); ++i)
    rv[i] = deg(angs[i]);

  return rv;
}

std::array<double, 6> rads(const std::array<double, 6>& angs)
{
  std::array<double, 6> rv;
  for (size_t i = 0; i < angs.size(); ++i)
    rv[i] = rad(angs[i]);

  return rv;
}

std::array<double, 6> mmdeg_pose(const std::array<double, 6>& pose)
{
  std::array<double, 6> rv;
  for (size_t i = 0; i < 3; ++i)
    rv[i] = 1000.0 * pose[i];

  for (size_t i = 3; i < 6; ++i)
    rv[i] = deg(pose[i]);

  return rv;
}

std::array<double, 6> m_rad_pose(const std::array<double, 6>& pose)
{
  std::array<double, 6> rv;
  for (size_t i = 0; i < 3; ++i)
    rv[i] = 0.001 * pose[i];

  for (size_t i = 3; i < 6; ++i)
    rv[i] = rad(pose[i]);

  return rv;
}

std::string set_tag(int tag, int wait)
{
  std::stringstream ss;
  ss << "QueueTag(" << tag << "," << wait << ")";
  return ss.str();
}

std::string set_wait_tag(int tag, int timeout_ms)
{
  std::stringstream ss;
  ss << "WaitQueueTag(" << tag << "," << timeout_ms << ")";
  return ss.str();
}

std::string set_io(TmIOModule module, TmIOType type, int pin, float state)
{
  static const std::array<std::string, 2> IO_MODULE_NAME = {"ControlBox", "EndModule"};
  static const std::array<std::string, 6> IO_TYPE_NAME = {"DI", "DO", "InstantDO", "AI", "AO", "InstantAO"};

  std::string script =
      "IO[" + IO_MODULE_NAME[int(module)] + "]." + IO_TYPE_NAME[int(type)] + "[" + std::to_string(pin) + "]=";
  if (type == TmIOType::DI || type == TmIOType::DO || type == TmIOType::InstantDO)
  {
    if (state == 0.0f)
    {
      script += "0";
    }
    else
    {
      script += "1";
    }
  }
  else
  {
    script += std::to_string(state);
  }
  return script;
}

std::string set_joint_pos_PTP(const std::array<double, 6>& angs, int vel_percent, double acc_time, int blend_percent,
                              bool fine_goal, int precision)
{
  auto angs_deg = degs(angs);
  const int acct_ms = int(1000.0 * acc_time);
  std::stringstream ss;
  ss << std::fixed << std::setprecision(precision);
  ss << "PTP(\"JPP\",";
  for (auto& value : angs_deg)
  {
    ss << value << ",";
  }
  ss << vel_percent << "," << acct_ms << "," << blend_percent << ",";
  ss << std::boolalpha << fine_goal << ")";
  return ss.str();
}

std::string set_tool_pose_PTP(const std::array<double, 6>& pose, int vel_percent, double acc_time, int blend_percent,
                              bool fine_goal, int precision)
{
  auto pose_mmdeg = mmdeg_pose(pose);
  const int acct_ms = int(1000.0 * acc_time);
  std::stringstream ss;
  ss << std::fixed << std::setprecision(precision);
  ss << "PTP(\"CPP\",";
  for (auto& value : pose_mmdeg)
  {
    ss << value << ",";
  }
  ss << vel_percent << "," << acct_ms << "," << blend_percent << ",";
  ss << std::boolalpha << fine_goal << ")";
  return ss.str();
}

std::string set_tool_pose_Line(const std::array<double, 6>& pose, double vel, double acc_time, int blend_percent,
                               bool fine_goal, int precision)
{
  auto pose_mmdeg = mmdeg_pose(pose);
  const int vel_mm = int(1000.0 * vel);
  const int acct_ms = int(1000.0 * acc_time);
  std::stringstream ss;
  ss << std::fixed << std::setprecision(precision);
  ss << "Line(\"CAP\",";
  for (auto& value : pose_mmdeg)
  {
    ss << value << ",";
  }
  ss << vel_mm << "," << acct_ms << "," << blend_percent << ",";
  ss << std::boolalpha << fine_goal << ")";
  return ss.str();
}

std::string set_pvt_point(TmPvtMode mode, double t, const std::array<double, 6>& pos, const std::array<double, 6>& vel,
                          int precision)
{
  std::stringstream ss;
  ss << std::fixed << std::setprecision(precision);
  ss << "PVTPoint(";
  if (mode == TmPvtMode::Joint)
  {
    for (auto& value : pos)
    {
      ss << deg(value) << ",";
    }
    for (auto& value : vel)
    {
      ss << deg(value) << ",";
    }
  }
  else
  {
    auto pv = mmdeg_pose(pos);
    for (auto& value : pv)
    {
      ss << value << ",";
    }
    auto vv = mmdeg_pose(vel);
    for (auto& value : vv)
    {
      ss << value << ",";
    }
  }
  ss << t << ")";
  return ss.str();
}

std::string set_pvt_traj(const TmPvtTraj& pvts, int precision)
{
  std::stringstream ss;
  ss << std::fixed << std::setprecision(precision);
  if (pvts.mode == TmPvtMode::Joint)
  {
    ss << "PVTEnter(0)\r\n";
    for (auto& point : pvts.points)
    {
      ss << "PVTPoint(";
      for (auto& value : point.positions)
      {
        ss << deg(value) << ",";
      }
      for (auto& value : point.velocities)
      {
        ss << deg(value) << ",";
      }
      ss << point.time << ")\r\n";
    }
  }
  else
  {
    ss << "PVTEnter(1)\r\n";
    for (auto& point : pvts.points)
    {
      ss << "PVTPoint(";
      auto pv = mmdeg_pose(point.positions);
      for (auto& value : pv)
      {
        ss << value << ",";
      }
      auto vv = mmdeg_pose(point.velocities);
      for (auto& value : vv)
      {
        ss << value << ",";
      }
      ss << point.time << ")\r\n";
    }
  }
  ss << "PVTExit()";
  return ss.str();
}

std::string set_vel_mode_start(VelMode mode, double timeout_zero_vel, double timeout_stop)
{
  if (mode == VelMode::Joint)
  {
    return "ContinueVJog()";
  }
  else
  {
    return "ContinueVLine(" + std::to_string((int)(1000.0 * timeout_zero_vel)) + ", " +
           std::to_string((int)(1000.0 * timeout_stop)) + ")";
  }
}

std::string set_vel_mode_target(VelMode mode, const std::array<double, 6>& vel, int precision)
{
  std::stringstream ss;
  ss << std::fixed << std::setprecision(precision);
  if (mode == VelMode::Joint)
  {
    ss << "SetContinueVJog(";
    if (!vel.empty())
    {
      size_t i = 0;
      for (; i < vel.size() - 1; ++i)
      {
        ss << deg(vel[i]) << ",";
      }
      ss << deg(vel[i]);
    }
    ss << ")";
  }
  else
  {
    ss << "SetContinueVLine(";
    if (vel.size() >= 3)
    {
      size_t i = 0;
      for (; i < 3; ++i)
      {
        ss << (1000.0 * vel[i]) << ",";
      }
      for (; i < vel.size() - 1; ++i)
      {
        ss << deg(vel[i]) << ",";
      }
      ss << deg(vel[i]);
    }
    ss << ")";
  }
  return ss.str();
}

}  // namespace tm_command
