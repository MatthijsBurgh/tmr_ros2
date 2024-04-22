#pragma once
#include "tm_svr_communication.h"
#include "tm_sct_communication.h"
#include "tm_command.h"

#include <string>

class TmDriver
{
public:
  TmSvrCommunication svr;
  TmSctCommunication sct;
  TmRobotState& state;

  const TmCommRC RC_OK = TmCommRC::OK;

private:
  bool is_positions_match(const TmPvtPoint& point, double eps) const;

  std::condition_variable* svr_cv_ = nullptr;
  std::condition_variable* sct_cv_ = nullptr;
  // bool has_svr_thrd_ = false;
  // bool has_sct_thrd_ = false;

  bool is_executing_traj_ = false;

  inline bool is_exec_pvt_traj() const
  {
    return is_executing_traj_;
  }

  ////////////////////////////////
  // tm_driver Param.
  ////////////////////////////////

  double max_velocity_ = M_PI;
  double max_tcp_speed_ = 1.5;
  double max_payload_ = 4.0;
  bool is_on_listen_node_ = false;
  bool connect_recovery_is_halt_ = false;  // false: do the recovery; true: stop the recovery

public:
  explicit TmDriver(const std::string& ip);
  explicit TmDriver(const std::string& ip, std::condition_variable* psvr_cv, std::condition_variable* psct_cv);

  ~TmDriver() = default;

  // start: connect to server, run project, connect to listen node
  bool start(int timeout_ms = -1, bool stick_play = true);

  // halt: disconnect to listen node, stop project, disconnect to server
  void halt();

  ////////////////////////////////
  // tm_driver Param.
  ////////////////////////////////

  void set_this_max_velocity(double max_vel)
  {
    max_velocity_ = max_vel;
  }

  void set_this_max_tcp_speed(double max_spd)
  {
    max_tcp_speed_ = max_spd;
  }

  void set_this_max_payload(double payload)
  {
    max_payload_ = payload;
  }

  ////////////////////////////////
  // SVR Robot Function (write_XXX)
  ////////////////////////////////

  ////////////////////////////////
  // SCT Robot Function (set_XXX)
  ////////////////////////////////
  inline bool is_on_listen_node() const
  {
    return is_on_listen_node_;
  }

  bool script_exit(const std::string& id = "Exit");
  bool set_tag(int tag, int wait = 0, const std::string& id = "Tag");
  bool set_wait_tag(int tag, int timeout_ms = 0, const std::string& id = "WaitTag");
  bool set_stop(const std::string& id = "Stop");
  bool set_pause(const std::string& id = "Pause");
  bool set_resume(const std::string& id = "Resume");

  // enum class IOModule { ControlBox, EndEffector };
  // enum class IOType { DI, DO, InstantDO, AI, AO, InstantAO };
  bool set_io(TmIOModule module, TmIOType type, int pin, float state, const std::string& id = "io");
  bool set_joint_pos_PTP(const std::array<double, 6>& angs, double vel, double acc_time, int blend_percent,
                         bool fine_goal = false, const std::string& id = "PTPJ");
  bool set_tool_pose_PTP(const std::array<double, 6>& pose, double vel, double acc_time, int blend_percent,
                         bool fine_goal = false, const std::string& id = "PTPT");
  bool set_tool_pose_Line(const std::array<double, 6>& pose, double vel, double acc_time, int blend_percent,
                          bool fine_goal = false, const std::string& id = "Line");
  // set_tool_pose_PLINE

  //
  // PVT Trajectory
  //

  bool set_pvt_enter(TmPvtMode mode, const std::string& id = "PvtEnter");
  bool set_pvt_exit(const std::string& id = "PvtExit");
  bool set_pvt_point(TmPvtMode mode, double t, const std::array<double, 6>& pos, const std::array<double, 6>& vel,
                     const std::string& id = "PvtPt");
  bool set_pvt_point(TmPvtMode mode, const TmPvtPoint& point, const std::string& id = "PvtPt");

  bool set_pvt_traj(const TmPvtTraj& pvts, const std::string& id = "PvtTraj");

  bool run_pvt_traj(const TmPvtTraj& pvts, double goal_time_tolerance = 0.0);

  inline void start_pvt_traj()
  {
    is_executing_traj_ = true;
  }

  inline void stop_pvt_traj()
  {
    is_executing_traj_ = false;
  }

  void back_to_listen_node();
  bool get_connect_recovery_guide();
  void set_connect_recovery_guide(bool is_halt);

  void cubic_interp(TmPvtPoint& p, const TmPvtPoint& p0, const TmPvtPoint& p1, double t);
  bool fake_run_pvt_traj(const TmPvtTraj& pvts);

  bool set_vel_mode_start(VelMode mode, double timeout_zero_vel, double timeout_stop,
                          const std::string& id = "VModeStart");
  bool set_vel_mode_stop(const std::string& id = "VModeStop");
  bool set_vel_mode_target(VelMode mode, const std::array<double, 6>& vel, const std::string& id = "VModeTrgt");
};
