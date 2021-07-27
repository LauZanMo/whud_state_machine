/**
 * @file WhudSwitch.cpp
 * @author Liuzhihao (liuzhihao@whu.edu.cn)
 * @brief WhudSwitch plugin
 * @version 1.0
 * @date 2021-06-22
 *
 * @copyright Copyright (c) 2021 WHU-Drones
 *
 */
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdlib.h>

#include "DataStructure.hpp"
#include "StateMachinePlugin.hpp"

using namespace std;

namespace whud_state_machine {
class WhudSwitch : public PluginBase {
public:
  WhudSwitch() : PluginBase(), nh_("~whud_switch") {}

  void OnInit(MavRosPublisher &mavros_pub) {
    PluginBase::OnInit(mavros_pub);
  }

  bool SetTask(ros::V_string param) {
    PluginBase::SetTask(param);
    if (nh_.hasParam(param[0])) {
      switch_name_ = param[0];
      nh_.setParam(switch_name_, false);
      return true;
    } else
      return false;
  }

  virtual void TaskSpin() override {
    nh_.getParam(switch_name_, switch_state_);
    if (switch_state_) task_status_ = TaskStatus::DONE;
  }

  virtual void StopTask() override {}

private:
  ros::NodeHandle nh_;
  bool switch_state_ = false;
  std::string switch_name_;
};

}  // namespace whud_state_machine

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(whud_state_machine::WhudSwitch,
                       whud_state_machine::PluginBase)
