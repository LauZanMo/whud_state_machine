/**
 * @file StateMachinePlugin.hpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @brief State machine plugin base
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
  PluginBase() : base_nh_("~") {}
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
  inline bool FinishDelay() {
    return (finish_delay_counter_++ >= finish_delay_time_ * loop_frequency_);
  }

  virtual void OnInit(MavRosPublisher& mavros_pub) {
    base_nh_.param<int>("loop_frequency", loop_frequency_, 10);
    mavros_pub_ = &mavros_pub;
  }

  virtual bool SetTask(ros::V_string param) {
    task_status_ = TaskStatus::RUN;
    interrupt_signal_ = false;
    finish_delay_counter_ = 0;
    SetFinishDelay(0);
  }

  virtual void TaskSpin() = 0;
  virtual void StopTask() = 0;

protected:
  ros::NodeHandle base_nh_;

  MavRosPublisher* mavros_pub_ = nullptr;
  TaskStatus task_status_;
  bool interrupt_signal_;
  bool control_flag_;

  int loop_frequency_;
  float finish_delay_time_ = 0;
  int finish_delay_counter_;

  inline void SetFinishDelay(float time) {
    finish_delay_time_ = time;
  }
};

}  // namespace whud_state_machine
