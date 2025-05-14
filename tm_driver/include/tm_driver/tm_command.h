#pragma once

#include <array>
#define _USE_MATH_DEFINES
#include <cmath>
#include <string>
#include <vector>

enum class TmIOModule
{
  ControlBox,
  EndEffector
};
enum class TmIOType
{
  DI,
  DO,
  InstantDO,
  AI,
  AO,
  InstantAO
};

enum class TmPvtMode
{
  Joint,
  Tool
};

struct TmPvtPoint
{
  double time;
  std::array<double, 6> positions;
  std::array<double, 6> velocities;
};

struct TmPvtTraj
{
  TmPvtMode mode;
  std::vector<TmPvtPoint> points;
  double total_time;
};

enum class VelMode
{
  Joint,
  Tool
};

namespace tm_command
{
////////////////////////////////
// Unit conversion
////////////////////////////////

inline double deg(double ang)
{
  return (180.0 / M_PI) * ang;
}

inline double rad(double ang)
{
  return (M_PI / 180.0) * ang;
}

std::array<double, 6> degs(const std::array<double, 6>& angs);

std::array<double, 6> rads(const std::array<double, 6>& angs);

std::array<double, 6> mmdeg_pose(const std::array<double, 6>& pose);

std::array<double, 6> m_rad_pose(const std::array<double, 6>& pose);

////////////////////////////////
// Functions
////////////////////////////////

/*
  Leaving external control mode.
  More Detail please refer to the TM_Robot_Expression.pdf Chapter 8.2 */
inline std::string script_exit()
{
  return "Exit()";
}

// More details please refer to the TM_Robot_Expression.pdf Chapter 9.1
std::string set_tag(int tag, int wait = 0);

// More details please refer to the TM_Robot_Expression.pdf Chapter 9.2
std::string set_wait_tag(int tag, int timeout_ms = 0);

/* Stopping robot motion and clear buffer.
  More details please refer to the TM_Robot_Expression.pdf Chapter 9.3 */
inline std::string set_stop()
{
  return "StopAndClearBuffer()";
}

/* Pausing pjoject and robot motion.
  More details please refer to the TM_Robot_Expression.pdf Chapter 9.4 */
inline std::string set_pause()
{
  return "Pause()";
}

/* Resuming project operation and robot motion.
  More details please refer to the TM_Robot_Expression.pdf Chapter 9.5 */
inline std::string set_resume()
{
  return "Resume()";
}

/* Triggering IO high/low.
  TmIOModule type: ControlBox, EndEffector
  TmIOType   type: DI, DO, InstantDO, AI, AO, InstantAO
  More details please refer to the TM_Robot_Expression.pdf Chapter 7.5*/
std::string set_io(TmIOModule module, TmIOType type, int pin, float state);

/* PTP(point to point) motion with angle calculation.
  The motion is mot always a straight line.
  More details please refer to the TM_Robot_Expression.pdf Chapter 9.6 */
std::string set_joint_pos_PTP(const std::array<double, 6>& angs, int vel_percent, double acc_time, int blend_percent,
                              bool fine_goal, int precision = 5);

/* PTP(point to point) motion with tool calculation.
  The motion is mot always a straight line.
  More details please refer to the TM_Robot_Expression.pdf Chapter 9.6 */
std::string set_tool_pose_PTP(const std::array<double, 6>& pose, int vel_percent, double acc_time, int blend_percent,
                              bool fine_goal, int precision = 5);

/* Linear motion with tool calculation.
  More details please refer to the TM_Robot_Expression.pdf Chapter 9.7 */
std::string set_tool_pose_Line(const std::array<double, 6>& pose, double vel, double acc_time, int blend_percent,
                               bool fine_goal, int precision = 5);

/* PVT start.
  More details please refer to the TM_Robot_Expression.pdf Chapter 9.16 */
inline std::string set_pvt_enter(int mode)
{
  return "PVTEnter(" + std::to_string(mode) + ")";
}

/* PVT leave.
  More details please refer to the TM_Robot_Expression.pdf Chapter 9.17 */
inline std::string set_pvt_exit()
{
  return "PVTExit()";
}

/* Setting target, velocity and time.
  TmPvtMode: Joint, Tool
  More details please refer to the TM_Robot_Expression.pdf Chapter 9.18 */
std::string set_pvt_point(TmPvtMode mode, double t, const std::array<double, 6>& pos, const std::array<double, 6>& vel,
                          int precision = 5);

/* Setting target, velocity and time.
  TmPvtMode: Joint, Tool
  TmPvtPoint: time ,positions and velocities;
  More details please refer to the TM_Robot_Expression.pdf Chapter 9.18 */
inline std::string set_pvt_point(TmPvtMode mode, const TmPvtPoint& point, int precision = 5)
{
  return set_pvt_point(mode, point.time, point.positions, point.velocities, precision);
}

/* Setting target, velocity and total time.
  TmPvtTraj: TmPvtMode mode
  points: target
  total_time: total time
  More details please refer to the TM_Robot_Expression.pdf Chapter 9.18 */
std::string set_pvt_traj(const TmPvtTraj& pvts, int precision = 5);

std::string set_vel_mode_start(VelMode mode, double timeout_zero_vel, double timeout_stop);

inline std::string set_vel_mode_stop()
{
  return "StopContinueVmode()";
}

std::string set_vel_mode_target(VelMode mode, const std::array<double, 6>& vel, int precision = 5);
};  // namespace tm_command
