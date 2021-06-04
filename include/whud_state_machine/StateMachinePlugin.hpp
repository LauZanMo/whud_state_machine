/**
 * @file StateMachinePlugin.hpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @brief 
 * @version 1.0
 * @date 2021-06-04
 * 
 * @copyright Copyright (c) 2021 WHU-Drones
 * 
 */
#pragma once

#include <ros/ros.h>

#include "DataStructure.hpp"

namespace whud_state_machine {

class PluginBase {
public:
  PluginBase() {}
  PluginBase(const PluginBase&) = delete;
  ~PluginBase() {}

  inline TaskStatus GetTaskStatus() const {
    return task_status_;
  }
  inline bool GetInterruptSignal() const {
    return interrupt_signal_;
  }
  inline void EnableControl() {
    control_flag_ = true;
  }
  inline void DisableControl() {
    control_flag_ = false;
  }

  virtual void OnInit(MavRosPublisher& mavros_pub) {
    mavros_pub_ = &mavros_pub;
  }

  virtual bool SetTask(ros::V_string param) {
    task_status_ = TaskStatus::RUN;
    interrupt_signal_ = false;
  }

  virtual void TaskSpin() = 0;
  virtual void StopTask() = 0;

protected:
  MavRosPublisher* mavros_pub_ = nullptr;
  TaskStatus task_status_;
  bool interrupt_signal_;
  bool control_flag_;
};

}  // namespace whud_state_machine
