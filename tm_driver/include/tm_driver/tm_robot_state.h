#pragma once

#include "tm_driver_utilities.h"

#include <array>
#define _USE_MATH_DEFINES
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

struct TmRobotStateData
{
  unsigned char is_linked;
  unsigned char has_error;
  unsigned char is_proj_running;
  unsigned char is_proj_paused;
  unsigned char is_safeguard_A_triggered;
  unsigned char is_ESTOP_pressed;
  unsigned char camera_light;
  int error_code;
  std::array<float, 6> joint_angle;
  std::array<float, 6> flange_pose;
  std::array<float, 6> tool_pose;
  std::array<float, 6> tcp_frame;
  float tcp_mass;
  std::array<float, 6> tcp_cog;
  std::array<float, 3> tcp_force_vec;
  float tcp_force;
  std::array<float, 6> tcp_speed_vec;
  float tcp_speed;
  std::array<float, 6> joint_speed;
  std::array<float, 6> joint_torque;
  std::array<float, 6> joint_torque_average;
  std::array<float, 6> joint_torque_min;
  std::array<float, 6> joint_torque_max;
  int proj_speed;
  int ma_mode;
  char stick_play_pause;
  int robot_light;
  std::array<unsigned char, 16> ctrller_DO;
  std::array<unsigned char, 16> ctrller_DI;
  std::array<float, 2> ctrller_AO;
  std::array<float, 2> ctrller_AI;
  std::array<unsigned char, 4> ee_DO;
  std::array<unsigned char, 4> ee_DI;
  std::array<float, 2> ee_AO;
  std::array<float, 2> ee_AI;
};

class TmDataTable;

class TmRobotState
{
  friend class TmDataTable;

private:
  std::unique_ptr<TmDataTable> data_table_;

public:
  enum
  {
    DOF = 6
  };

private:
  std::mutex mtx;
  MultiThreadCache<TmRobotStateData> multiThreadCache;

private:
  TmRobotStateData tmRobotStateDataFromEthernet;
  bool is_data_table_correct_ = false;

private:
  TmRobotStateData tmRobotStateDataToPublish;

  TmCommRC _receive_state;

private:
  std::function<size_t(void*, const char*, size_t)> _f_deserialize_item[2];
  std::function<size_t(const char*, size_t, bool)> _f_deserialize;

  struct ItemUpdate
  {
    void* dst;
    size_t func;

    enum
    {
      SKIP,
      UPDATE
    };
  };

  std::vector<ItemUpdate> _item_updates;

public:
  TmRobotState();
  ~TmRobotState();

  inline bool is_linked() const
  {
    return tmRobotStateDataToPublish.is_linked;
  }

  inline bool has_error() const
  {
    return tmRobotStateDataToPublish.has_error;
  }

  inline bool is_data_table_correct() const
  {
    return is_data_table_correct_;
  }

  inline bool is_project_running() const
  {
    return tmRobotStateDataToPublish.is_proj_running;
  }

  inline bool is_project_paused() const
  {
    return tmRobotStateDataToPublish.is_proj_paused;
  }

  inline bool is_safeguard_A() const
  {
    return tmRobotStateDataToPublish.is_safeguard_A_triggered;
  }

  inline bool is_EStop() const
  {
    return tmRobotStateDataToPublish.is_ESTOP_pressed;
  }

  inline bool camera_light() const
  {
    return tmRobotStateDataToPublish.camera_light;
  }  // R/W

  inline int error_code() const
  {
    return tmRobotStateDataToPublish.error_code;
  }

  static inline std::string error_content()
  {
    return "";
  }

  std::array<double, 6> flange_pose() const
  {
    std::array<double, 6> flangePose;
    si_pose(flangePose, tmRobotStateDataToPublish.flange_pose.data(), 6);
    return flangePose;
  }

  std::array<double, 6> joint_angle() const
  {
    return rads(tmRobotStateDataToPublish.joint_angle.data(), 6);
  }

  std::array<double, 6> tool_pose() const
  {
    std::array<double, 6> toolPose;
    si_pose(toolPose, tmRobotStateDataToPublish.tool_pose.data(), 6);
    return toolPose;
  }

  std::array<double, 3> tcp_force_vec() const
  {
    std::array<double, 3> tcpForceVec;
    std::copy(tmRobotStateDataToPublish.tcp_force_vec.begin(), tmRobotStateDataToPublish.tcp_force_vec.end(),
              tcpForceVec.begin());
    return tcpForceVec;
  }

  inline double tcp_force() const
  {
    return tmRobotStateDataToPublish.tcp_force;
  }

  std::array<double, 6> tcp_speed_vec() const
  {
    std::array<double, 6> tcpSpeedVec;
    si_pose(tcpSpeedVec, tmRobotStateDataToPublish.tcp_speed_vec.data(), 6);
    return tcpSpeedVec;
  }

  inline double tcp_speed() const
  {
    return tmRobotStateDataToPublish.tcp_speed;
  }

  std::array<double, 6> joint_speed() const
  {
    return rads(tmRobotStateDataToPublish.joint_speed.data(), 6);
  }

  std::array<double, 6> joint_torque() const
  {
    return meters(tmRobotStateDataToPublish.joint_torque.data(), 6);
  }

  std::array<double, 6> joint_torque_average() const
  {
    return meters(tmRobotStateDataToPublish.joint_torque_average.data(), 6);
  }

  std::array<double, 6> joint_torque_min() const
  {
    return meters(tmRobotStateDataToPublish.joint_torque_min.data(), 6);
  }

  std::array<double, 6> joint_torque_max() const
  {
    return meters(tmRobotStateDataToPublish.joint_torque_max.data(), 6);
  }

  inline int project_speed() const
  {
    return tmRobotStateDataToPublish.proj_speed;
  }

  inline int ma_mode() const
  {
    return tmRobotStateDataToPublish.ma_mode;
  }

  inline bool stick_play_pause() const
  {
    return tmRobotStateDataToPublish.stick_play_pause;
  }  // R/W

  inline int robot_light() const
  {
    return tmRobotStateDataToPublish.robot_light;
  }  // R/W

  std::array<bool, 16> ctrller_DO() const
  {
    std::array<bool, 16> ctrllerDO;
    std::copy(tmRobotStateDataToPublish.ctrller_DO.begin(), tmRobotStateDataToPublish.ctrller_DO.end(),
              ctrllerDO.begin());
    return ctrllerDO;
  }

  std::array<bool, 16> ctrller_DI() const
  {
    std::array<bool, 16> ctrllerDI;
    std::copy(tmRobotStateDataToPublish.ctrller_DI.begin(), tmRobotStateDataToPublish.ctrller_DI.end(),
              ctrllerDI.begin());
    return ctrllerDI;
  }

  std::array<float, 2> ctrller_AO() const
  {
    std::array<float, 2> ctrllerAO;
    std::copy(tmRobotStateDataToPublish.ctrller_AO.begin(), tmRobotStateDataToPublish.ctrller_AO.end(),
              ctrllerAO.begin());
    return ctrllerAO;
  }

  std::array<float, 2> ctrller_AI() const
  {
    std::array<float, 2> ctrllerAI;
    std::copy(tmRobotStateDataToPublish.ctrller_AI.begin(), tmRobotStateDataToPublish.ctrller_AI.end(),
              ctrllerAI.begin());
    return ctrllerAI;
  }

  std::array<bool, 4> ee_DO() const
  {
    std::array<bool, 4> eeDO;
    std::copy(tmRobotStateDataToPublish.ee_DO.begin(), tmRobotStateDataToPublish.ee_DO.end(), eeDO.begin());
    return eeDO;
  }

  std::array<bool, 4> ee_DI() const
  {
    std::array<bool, 4> eeDI;
    std::copy(tmRobotStateDataToPublish.ee_DI.begin(), tmRobotStateDataToPublish.ee_DI.end(), eeDI.begin());
    return eeDI;
  }

  std::array<float, 2> ee_AO()
  {
    std::array<float, 2> eeAO;
    std::copy(tmRobotStateDataToPublish.ee_AO.begin(), tmRobotStateDataToPublish.ee_AO.end(), eeAO.begin());
    return eeAO;
  }

  std::array<float, 2> ee_AI() const
  {
    std::array<float, 2> eeAI;
    std::copy(tmRobotStateDataToPublish.ee_AI.begin(), tmRobotStateDataToPublish.ee_AI.end(), eeAI.begin());
    return eeAI;
  }

  inline TmCommRC get_receive_state() const
  {
    return _receive_state;
  }

  void set_fake_joint_states(const std::array<double, 6>& pos, const std::array<double, 6>& vel,
                             const std::array<double, 6>& tor);

public:
  void mtx_lock()
  {
    mtx.lock();
  }

  void mtx_unlock()
  {
    mtx.unlock();
  }

  std::array<double, 3> mtx_tcp_force_vec();
  std::array<double, 6> mtx_tcp_speed_vec();

  std::array<double, 6> mtx_joint_speed();
  std::array<double, 6> mtx_joint_torque();

  void mtx_set_joint_states(const std::array<double, 6>& pos, const std::array<double, 6>& vel,
                            const std::array<double, 6>& tor)
  {
    std::lock_guard<std::mutex> lck(mtx);
    set_fake_joint_states(pos, vel, tor);
  }

private:
  static double meter(double mm)
  {
    return 0.001 * mm;
  }

  static double rad(double ang)
  {
    return (M_PI / 180.0) * ang;
  }

  template<typename T>
  static std::array<double, 6> meters(const T* mms, size_t size)
  {
    std::array<double, 6> rv;
    for (size_t i = 0; i < size; ++i)
    {
      rv[i] = meter(static_cast<double>(mms[i]));
    }
    return rv;
  }

  template<typename T>
  static std::array<double, 6> rads(const T* degs, size_t size)
  {
    std::array<double, 6> rv;
    for (size_t i = 0; i < size; ++i)
    {
      rv[i] = rad(static_cast<double>(degs[i]));
    }
    return rv;
  }

  template<typename T>
  static void si_pose(std::array<double, 6>& dst, T* src, size_t size = 6)
  {
    for (size_t i = 0; i < 3; ++i)
    {
      dst[i] = meter(static_cast<double>(src[i]));
    }
    for (size_t i = 3; i < size; ++i)
    {
      dst[i] = rad(static_cast<double>(src[i]));
    }
  }

private:
  static size_t _deserialize_skip(void* dst, const char* data, size_t offset);
  static size_t _deserialize_copy_wo_check(void* dst, const char* data, size_t offset);
  size_t _deserialize_first_time(const char* data, size_t size);
  size_t _deserialize(const char* data, size_t size);

public:
  size_t deserialize(const char* data, size_t size)
  {
    return _f_deserialize(data, size, false);
  }

  size_t mtx_deserialize(const char* data, size_t size)
  {
    return _f_deserialize(data, size, true);
  }

  void set_receive_state(TmCommRC state);
  void update_tm_robot_publish_state();
  void print();
};
