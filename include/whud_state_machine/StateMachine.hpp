#pragma once

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

#include <vector>

#include "DataStructure.hpp"
#include "StateMachinePlugin.hpp"

using namespace std;

namespace whud_state_machine {

using PluginPtr = boost::shared_ptr<PluginBase>;

class StateMachine {
 public:
  StateMachine();
  ~StateMachine();
  void Run();

 private:
  // state machine
  ros::NodeHandle nh_;
  ros::Timer loop_timer_;
  int loop_frequency_;
  int state_machine_threads_;
  MavRosPublisher mavros_pub_;

  // tasks
  StateMachineStatus state_machine_status_;
  ros::V_string main_task_list_;
  vector<MainTask> main_task_vector_;
  vector<MainTask>::iterator main_task_iterator_;
  InterruptTask current_interrupt_task_;

  bool disable_interrupt_flag_ = false;
  ros::Time main_task_begin_time_;
  ros::Time interrupt_task_begin_time_;
  bool last_interrupt_flag_ = false;

  // plugins
  PluginPtr current_main_task_plugin_;
  PluginPtr current_interrupt_task_plugin_;
  pluginlib::ClassLoader<PluginBase> plugin_loader_;
  map<string, PluginPtr> plugin_map_;

  void LoopTimerCb(const ros::TimerEvent &event);
  void SetTask(const ros::TimerEvent &event);
  void TaskSpin(const ros::TimerEvent &event);
  void CheckLoopStatus();

  void LoadPlugin(std::string &plugin_name);
  void LoadMainTask(std::string &task_name);
  void SetInterruptTask(std::string &task_name);
};

}  // namespace whud_state_machine
