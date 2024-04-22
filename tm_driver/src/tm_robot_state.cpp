#include "tm_driver/tm_driver_utilities.h"
#include "tm_driver/tm_robot_state.h"
#include "tm_driver/tm_print.h"

#include <array>
#define _USE_MATH_DEFINES
#include <cmath>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>

class TmDataTable
{
public:
  struct Item
  {
  public:
    void* dst;
    bool required;
    bool checked;

    enum
    {
      NOT_REQUIRE = 0,
      REQUIRED = 1
    };

    Item() : dst(nullptr), required(true), checked(false){};
    Item(void* d) : dst(d), required(true), checked(false){};
    Item(void* d, bool r) : dst(d), required(r), checked(false){};
  };

private:
  std::map<std::string, Item> item_map_;

public:
  TmDataTable(TmRobotState* rs)
  {
    print_debug("Create DataTable");

    item_map_.clear();
    //_item_map[""] = { Item:, &rs- };
    item_map_["Robot_Link"] = {&rs->tmRobotStateDataFromEthernet.is_linked};
    item_map_["Robot_Error"] = {&rs->tmRobotStateDataFromEthernet.has_error};
    item_map_["Project_Run"] = {&rs->tmRobotStateDataFromEthernet.is_proj_running};
    item_map_["Project_Pause"] = {&rs->tmRobotStateDataFromEthernet.is_proj_paused};
    item_map_["Safeguard_A"] = {&rs->tmRobotStateDataFromEthernet.is_safeguard_A_triggered, Item::NOT_REQUIRE};
    item_map_["ESTOP"] = {&rs->tmRobotStateDataFromEthernet.is_ESTOP_pressed};
    item_map_["Camera_Light"] = {&rs->tmRobotStateDataFromEthernet.camera_light};
    item_map_["Error_Code"] = {&rs->tmRobotStateDataFromEthernet.error_code};
    item_map_["Joint_Angle"] = {&rs->tmRobotStateDataFromEthernet.joint_angle};
    item_map_["Coord_Robot_Flange"] = {&rs->tmRobotStateDataFromEthernet.flange_pose};
    item_map_["Coord_Robot_Tool"] = {&rs->tmRobotStateDataFromEthernet.tool_pose};
    item_map_["TCP_Force"] = {&rs->tmRobotStateDataFromEthernet.tcp_force_vec};
    item_map_["TCP_Force3D"] = {&rs->tmRobotStateDataFromEthernet.tcp_force};
    item_map_["TCP_Speed"] = {&rs->tmRobotStateDataFromEthernet.tcp_speed_vec};
    item_map_["TCP_Speed3D"] = {&rs->tmRobotStateDataFromEthernet.tcp_speed};
    item_map_["Joint_Speed"] = {&rs->tmRobotStateDataFromEthernet.joint_speed};
    item_map_["Joint_Torque"] = {&rs->tmRobotStateDataFromEthernet.joint_torque};
    item_map_["Joint_Torque_Average"] = {&rs->tmRobotStateDataFromEthernet.joint_torque_average, Item::NOT_REQUIRE};
    item_map_["Joint_Torque_Min"] = {&rs->tmRobotStateDataFromEthernet.joint_torque_min, Item::NOT_REQUIRE};
    item_map_["Joint_Torque_Max"] = {&rs->tmRobotStateDataFromEthernet.joint_torque_max, Item::NOT_REQUIRE};
    item_map_["Project_Speed"] = {&rs->tmRobotStateDataFromEthernet.proj_speed};
    item_map_["MA_Mode"] = {&rs->tmRobotStateDataFromEthernet.ma_mode};
    item_map_["Robot_Light"] = {&rs->tmRobotStateDataFromEthernet.robot_light};
    item_map_["Ctrl_DO0"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DO[0]};
    item_map_["Ctrl_DO1"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DO[1]};
    item_map_["Ctrl_DO2"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DO[2]};
    item_map_["Ctrl_DO3"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DO[3]};
    item_map_["Ctrl_DO4"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DO[4]};
    item_map_["Ctrl_DO5"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DO[5]};
    item_map_["Ctrl_DO6"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DO[6]};
    item_map_["Ctrl_DO7"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DO[7]};
    item_map_["Ctrl_DO8"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DO[8], Item::NOT_REQUIRE};
    item_map_["Ctrl_DO9"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DO[9], Item::NOT_REQUIRE};
    item_map_["Ctrl_DO10"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DO[10], Item::NOT_REQUIRE};
    item_map_["Ctrl_DO11"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DO[11], Item::NOT_REQUIRE};
    item_map_["Ctrl_DO12"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DO[12], Item::NOT_REQUIRE};
    item_map_["Ctrl_DO13"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DO[13], Item::NOT_REQUIRE};
    item_map_["Ctrl_DO14"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DO[14], Item::NOT_REQUIRE};
    item_map_["Ctrl_DO15"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DO[15], Item::NOT_REQUIRE};
    item_map_["Ctrl_DI0"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DI[0]};
    item_map_["Ctrl_DI1"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DI[1]};
    item_map_["Ctrl_DI2"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DI[2]};
    item_map_["Ctrl_DI3"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DI[3]};
    item_map_["Ctrl_DI4"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DI[4]};
    item_map_["Ctrl_DI5"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DI[5]};
    item_map_["Ctrl_DI6"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DI[6]};
    item_map_["Ctrl_DI7"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DI[7]};
    item_map_["Ctrl_DI8"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DI[8], Item::NOT_REQUIRE};
    item_map_["Ctrl_DI9"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DI[9], Item::NOT_REQUIRE};
    item_map_["Ctrl_DI10"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DI[10], Item::NOT_REQUIRE};
    item_map_["Ctrl_DI11"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DI[11], Item::NOT_REQUIRE};
    item_map_["Ctrl_DI12"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DI[12], Item::NOT_REQUIRE};
    item_map_["Ctrl_DI13"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DI[13], Item::NOT_REQUIRE};
    item_map_["Ctrl_DI14"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DI[14], Item::NOT_REQUIRE};
    item_map_["Ctrl_DI15"] = {&rs->tmRobotStateDataFromEthernet.ctrller_DI[15], Item::NOT_REQUIRE};
    item_map_["Ctrl_AO0"] = {&rs->tmRobotStateDataFromEthernet.ctrller_AO[0]};
    item_map_["Ctrl_AO1"] = {&rs->tmRobotStateDataFromEthernet.ctrller_AO[1], Item::NOT_REQUIRE};
    item_map_["Ctrl_AI0"] = {&rs->tmRobotStateDataFromEthernet.ctrller_AI[0]};
    item_map_["Ctrl_AI1"] = {&rs->tmRobotStateDataFromEthernet.ctrller_AI[1], Item::NOT_REQUIRE};
    item_map_["End_DO0"] = {&rs->tmRobotStateDataFromEthernet.ee_DO[0]};
    item_map_["End_DO1"] = {&rs->tmRobotStateDataFromEthernet.ee_DO[1]};
    item_map_["End_DO2"] = {&rs->tmRobotStateDataFromEthernet.ee_DO[2]};
    item_map_["End_DO3"] = {&rs->tmRobotStateDataFromEthernet.ee_DO[3]};
    item_map_["End_DI0"] = {&rs->tmRobotStateDataFromEthernet.ee_DI[0]};
    item_map_["End_DI1"] = {&rs->tmRobotStateDataFromEthernet.ee_DI[1]};
    item_map_["End_DI2"] = {&rs->tmRobotStateDataFromEthernet.ee_DI[2]};
    item_map_["End_DI3"] = {&rs->tmRobotStateDataFromEthernet.ee_DI[3], Item::NOT_REQUIRE};
    item_map_["End_AO0"] = {&rs->tmRobotStateDataFromEthernet.ee_AO[0], Item::NOT_REQUIRE};
    item_map_["End_AO1"] = {&rs->tmRobotStateDataFromEthernet.ee_AO[1], Item::NOT_REQUIRE};
    item_map_["End_AI0"] = {&rs->tmRobotStateDataFromEthernet.ee_AI[0]};
    item_map_["End_AI1"] = {&rs->tmRobotStateDataFromEthernet.ee_AI[1], Item::NOT_REQUIRE};
  }

  std::map<std::string, Item>& get()
  {
    return item_map_;
  }

  std::map<std::string, Item>::iterator find(const std::string& name)
  {
    return item_map_.find(name);
  }

  std::map<std::string, Item>::iterator end()
  {
    return item_map_.end();
  }
};

TmRobotState::TmRobotState()
{
  print_debug("TmRobotState::TmRobotState");

  data_table_ = std::make_unique<TmDataTable>(this);

  _f_deserialize_item[0] =
      std::bind(&TmRobotState::_deserialize_skip, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
  _f_deserialize_item[1] = std::bind(&TmRobotState::_deserialize_copy_wo_check, std::placeholders::_1,
                                     std::placeholders::_2, std::placeholders::_3);

  _f_deserialize =
      std::bind(&TmRobotState::_deserialize_first_time, this, std::placeholders::_1, std::placeholders::_2);
}

TmRobotState::~TmRobotState()
{
  print_debug("TmRobotState::~TmRobotState");
}

void TmRobotState::set_fake_joint_states(const std::array<double, 6>& pos, const std::array<double, 6>& vel,
                                         const std::array<double, 6>& tor)
{
  for (size_t i = 0; i < 6; ++i)
  {
    tmRobotStateDataFromEthernet.joint_angle[i] = pos[i] * (180.0 / M_PI);
    tmRobotStateDataFromEthernet.joint_speed[i] = vel[i];
    tmRobotStateDataFromEthernet.joint_torque[i] = tor[i];
  }
  multiThreadCache.set_catch_data(tmRobotStateDataFromEthernet);
}

std::array<double, 3> TmRobotState::mtx_tcp_force_vec()
{
  const std::lock_guard<std::mutex> lck(mtx);
  return tcp_force_vec();
}

std::array<double, 6> TmRobotState::mtx_tcp_speed_vec()
{
  const std::lock_guard<std::mutex> lck(mtx);
  return tcp_speed_vec();
}

std::array<double, 6> TmRobotState::mtx_joint_speed()
{
  const std::lock_guard<std::mutex> lck(mtx);
  return joint_speed();
}

std::array<double, 6> TmRobotState::mtx_joint_torque()
{
  const std::lock_guard<std::mutex> lck(mtx);
  return joint_torque();
}

size_t TmRobotState::_deserialize_skip(void* dst, const char* data, size_t offset)
{
  size_t boffset = offset;
  unsigned short uslen;  // 2 bytes

  // skip item name
  memcpy(&uslen, data + boffset, 2);
  boffset += 2 + uslen;
  // skip item
  memcpy(&uslen, data + boffset, 2);
  boffset += 2 + uslen;

  if (dst) {}
  return boffset;
}

size_t TmRobotState::_deserialize_copy_wo_check(void* dst, const char* data, size_t offset)
{
  size_t boffset = offset;
  // size_t bsize = 2;
  unsigned short uslen;  // 2 bytes

  // skip item name
  memcpy(&uslen, data + boffset, 2);
  boffset += 2 + uslen;
  // item data length
  memcpy(&uslen, data + boffset, 2);
  boffset += 2;
  // item data
  // bsize = uslen;
  memcpy(dst, data + boffset, uslen);
  boffset += uslen;
  return boffset;
}

size_t TmRobotState::_deserialize_first_time(const char* data, size_t size)
{
  size_t boffset = 0;
  size_t count = 0;
  size_t check_count = 0;
  size_t skip_count = 0;
  unsigned short uslen = 0;  // 2 bytes
  std::string item_name;

  print_info("TM Flow DataTable Checked Item: ");
  _item_updates.clear();
  //_f_deserialize_item.clear();

  while (boffset < size && count < 100)
  {
    // item name length
    memcpy(&uslen, data + boffset, 2);
    boffset += 2;
    // item name
    item_name = std::string{data + boffset, uslen};
    boffset += uslen;

    ItemUpdate update{nullptr, ItemUpdate::SKIP};
    // std::function<size_t (void *, const char *, size_t)> func;
    auto iter = data_table_->find(item_name);
    if (iter != data_table_->end())
    {
      update.dst = iter->second.dst;
      update.func = ItemUpdate::UPDATE;
      // func = std::bind(&RobotState::_deserialize_copy_wo_check, iter->second.dst,
      // std::placeholders::_2, std::placeholders::_3);
      iter->second.checked = true;
      const std::string msg = "- " + item_name + " - checked";
      print_debug(msg.c_str());
      ++check_count;
    }
    else
    {
      // func = std::bind(&RobotState::_deserialize_skip, nullptr,
      // std::placeholders::_2, std::placeholders::_3);
      const std::string msg = "- " + item_name + " - skipped";
      print_debug(msg.c_str());
      ++skip_count;
    }
    _item_updates.push_back({update.dst, update.func});
    //_f_deserialize_item.push_back(func);

    // item data length
    memcpy(&uslen, data + boffset, 2);
    boffset += 2;
    if (update.func == ItemUpdate::SKIP)
    {
      // skip item
      boffset += uslen;
    }
    else
    {
      // item data
      memcpy(update.dst, data + boffset, uslen);
      boffset += uslen;
    }
    ++count;
  }

  is_data_table_correct_ = true;
  const std::string msg = "Total " + std::to_string(_item_updates.size()) + " items, " + std::to_string(check_count) +
                          " checked, " + std::to_string(skip_count) + " skipped";
  print_info(msg.c_str());

  multiThreadCache.set_catch_data(tmRobotStateDataFromEthernet);

  for (const auto& iter : data_table_->get())
  {
    if (iter.second.required && !iter.second.checked)
    {
      is_data_table_correct_ = false;
      const std::string msg = "Required item" + iter.first + " is NOT checked";
      print_error(msg.c_str());
    }
  }

  if (is_data_table_correct_)
  {
    print_info("data table is correct!");
  }
  else
  {
    print_error("data table is not correct!");
  }

  _f_deserialize = std::bind(&TmRobotState::_deserialize, this, std::placeholders::_1, std::placeholders::_2);

  return boffset;
}

size_t TmRobotState::_deserialize(const char* data, size_t size)
{
  size_t boffset = 0;

  for (auto& update : _item_updates)
  {
    boffset = _f_deserialize_item[update.func](update.dst, data, boffset);
  }

  multiThreadCache.set_catch_data(tmRobotStateDataFromEthernet);

  if (boffset > size) {}
  return boffset;
}

void TmRobotState::update_tm_robot_publish_state()
{
  tmRobotStateDataToPublish = multiThreadCache.get_catch_data();
}

void TmRobotState::set_receive_state(TmCommRC state)
{
  _receive_state = state;
}
