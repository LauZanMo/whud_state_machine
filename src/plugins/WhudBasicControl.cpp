/**
 * @file WhudBasicControl.cpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @brief whud_basic_control plugin
 * @version 1.0
 * @date 2021-06-04
 *
 * @copyright Copyright (c) 2021 WHU-Drones
 *
 */
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <stdlib.h>

#include "DataStructure.hpp"
#include "StateMachinePlugin.hpp"

using namespace std;

namespace whud_state_machine {
class WhudBasicControl : public PluginBase {
public:
  WhudBasicControl() : PluginBase(), nh_("~basic_control"){};

  enum Command { NONE = 0, TAKEOFF, LAND, HEIGHT_CONTROL, YAW_CONTROL };

  void OnInit(MavRosPublisher &mavros_pub) {
    mavros_pub_ = &mavros_pub;
  }

  bool SetTask(ros::V_string param) {
    PluginBase::SetTask(param);
    if (param[0] == "takeoff") {
      takeoff_.data.clear();
      // set z axis speed
      takeoff_.data.push_back(atof(param[1].c_str()));

      // set height
      takeoff_.data.push_back(atof(param[2].c_str()));

      command_ = Command::TAKEOFF;

    } else if (param[0] == "land") {
      // set z axis speed
      land_.data = atof(param[1].c_str());

      command_ = Command::LAND;

    } else if (param[0] == "height_control") {
      height_control_.data.clear();
      // set z axis speed
      height_control_.data.push_back(atof(param[1].c_str()));

      // set target height(relatively)
      height_control_.data.push_back(atof(param[2].c_str()));

      command_ = Command::HEIGHT_CONTROL;

    } else if (param[0] == "yaw_control") {
      yaw_control_.data.clear();
      // set target angle
      yaw_control_.data.push_back(atof(param[1].c_str()));

      // 0: absolute, 1: relative
      yaw_control_.data.push_back(atof(param[2].c_str()));

      command_ = Command::YAW_CONTROL;

    } else {
      command_ = Command::NONE;
    }
  }

  virtual void TaskSpin() override {
    if (mavros_pub_ != nullptr && control_flag_ == true) {
      int mavros_command, mavros_result;
      nh_.getParam("/mavros/whud_basic/ack_cmd_index", mavros_command);
      nh_.getParam("/mavros/whud_basic/ack_result", mavros_result);

      switch (command_) {
      case Command::TAKEOFF:
        if (mavros_command == 24 && mavros_result == 0)
          task_status_ = TaskStatus::DONE;
        else if (mavros_command != 24 && mavros_result != 5)
          mavros_pub_->takeoff_pub.publish(takeoff_);
        break;

      case Command::LAND:
        if (mavros_command == 23 && mavros_result == 0)
          task_status_ = TaskStatus::DONE;
        else if (mavros_command != 23 && mavros_result != 5)
          mavros_pub_->land_pub.publish(land_);
        break;

      case Command::HEIGHT_CONTROL:
        if (mavros_command == 113 && mavros_result == 0)
          task_status_ = TaskStatus::DONE;
        else if (mavros_command != 113 && mavros_result != 5)
          mavros_pub_->height_pub.publish(height_control_);
        break;

      case Command::YAW_CONTROL:
        if (mavros_command == 115 && mavros_result == 0)
          task_status_ = TaskStatus::DONE;
        else if (mavros_command != 115 && mavros_result != 5)
          mavros_pub_->yaw_pub.publish(yaw_control_);
        break;

      default:
        break;
      }
    }
  }

  virtual void StopTask() override {}

private:
  ros::NodeHandle nh_;
  Command command_;
  std_msgs::Float64MultiArray takeoff_;
  std_msgs::Float64 land_;
  std_msgs::Float64MultiArray height_control_;
  std_msgs::Float64MultiArray yaw_control_;
};

}  // namespace whud_state_machine

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(whud_state_machine::WhudBasicControl,
                       whud_state_machine::PluginBase)
