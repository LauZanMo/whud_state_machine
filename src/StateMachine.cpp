#include "StateMachine.hpp"

#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;

namespace whud_state_machine {

StateMachine::StateMachine()
    : nh_("~"),
      state_machine_status_(StateMachineStatus::FREE),
      plugin_loader_("whud_state_machine", "whud_state_machine::PluginBase") {
  nh_.param<int>("loop_frequency", loop_frequency_, 10);
  nh_.param<int>("state_machine_threads", state_machine_threads_, 4);
  nh_.getParam("main_task_list", main_task_list_);

  // init publisher
  mavros_pub_.takeoff_pub =
      nh_.advertise<std_msgs::Float64MultiArray>("takeoff_height", 5);
  mavros_pub_.height_pub =
      nh_.advertise<std_msgs::Float64MultiArray>("height", 5);
  mavros_pub_.land_pub = nh_.advertise<std_msgs::Float64>("land", 5);
  mavros_pub_.yaw_pub = nh_.advertise<std_msgs::Float64MultiArray>("yaw", 5);
  mavros_pub_.cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);
  mavros_pub_.conversion_pub = nh_.advertise<std_msgs::Bool>("conversion", 5);

  // load plugin
  for (auto &plugin_name : plugin_loader_.getDeclaredClasses())
    LoadPlugin(plugin_name);

  // load task
  for (auto &task_name : main_task_list_) LoadMainTask(task_name);

  // check main task empty
  if (main_task_list_.empty()) {
    state_machine_status_ = StateMachineStatus::END;
    ROS_WARN("Main task list is empty!");
  }

  // main task iterator initialize
  main_task_iterator_ = main_task_vector_.begin();
}

StateMachine::~StateMachine() {}

void StateMachine::Run() {
  ros::AsyncSpinner spinner(state_machine_threads_);

  loop_timer_ = nh_.createTimer(ros::Duration(1.0 / (double)loop_frequency_),
                                &StateMachine::LoopTimerCb, this);

  spinner.start();
  ros::waitForShutdown();
  ROS_INFO("Stopping state machine...");
  spinner.stop();
}

void StateMachine::LoopTimerCb(const ros::TimerEvent &event) {
  if (state_machine_status_ == StateMachineStatus::FREE)
    SetTask(event);
  else if (state_machine_status_ == StateMachineStatus::MAIN_TASK ||
           state_machine_status_ == StateMachineStatus::INTERRUPT_TASK)
    TaskSpin(event);

  CheckLoopStatus();
}

void StateMachine::SetTask(const ros::TimerEvent &event) {
  // set main task
  current_main_task_plugin_ = plugin_map_[main_task_iterator_->plugin_name];
  current_main_task_plugin_->SetTask(main_task_iterator_->param);
  ROS_INFO_STREAM("Current task is " + main_task_iterator_->task_name);

  // enable main plugin control
  current_main_task_plugin_->EnableControl();

  // set interrupt task
  if (disable_interrupt_flag_) {
    string name = "none";
    SetInterruptTask(name);

    // reset this flag
    disable_interrupt_flag_ = false;
  } else
    SetInterruptTask(main_task_iterator_->attach_name);

  // record main task begin time
  main_task_begin_time_ = event.current_expected.now().toSec();

  // set loop status
  state_machine_status_ = StateMachineStatus::MAIN_TASK;
}

void StateMachine::TaskSpin(const ros::TimerEvent &event) {
  // check interrupt plugin ptr
  if (current_interrupt_task_plugin_ != nullptr) {
    // check interrupt flag
    bool interrupt_flag = current_interrupt_task_plugin_->GetInterruptSignal();
    if (interrupt_flag) {
      // check last interrupt flag
      if (last_interrupt_flag_ != interrupt_flag) {
        ROS_INFO_STREAM("Current task is " + main_task_iterator_->attach_name);

        // record begin time
        interrupt_task_begin_time_ = event.current_expected.now().toSec();

        // set loop status
        state_machine_status_ = StateMachineStatus::INTERRUPT_TASK;

        // enable interrupt plugin control
        current_main_task_plugin_->DisableControl();
        current_interrupt_task_plugin_->EnableControl();
      }

      // interrupt task spin
      current_interrupt_task_plugin_->TaskSpin();

      // check interrupt task timeout
      if (event.current_expected.now().toSec() - interrupt_task_begin_time_ >
          current_interrupt_task_.delay_timeout)
        state_machine_status_ = StateMachineStatus::INTERRUPT_TASK_TIMEOUT;

    } else {
      // main task spin
      current_main_task_plugin_->TaskSpin();

      // check main task timeout
      if (event.current_expected.now().toSec() - main_task_begin_time_ >
          main_task_iterator_->delay_timeout)
        state_machine_status_ = StateMachineStatus::MAIN_TASK_TIMEOUT;
    }

    // record last interrupt flag
    last_interrupt_flag_ = interrupt_flag;

  } else {
    // main task spin
    current_main_task_plugin_->TaskSpin();

    double a = event.current_expected.now().toSec() - main_task_begin_time_;

    // check main task timeout
    if (event.current_expected.now().toSec() - main_task_begin_time_ >
        main_task_iterator_->delay_timeout)
      state_machine_status_ = StateMachineStatus::MAIN_TASK_TIMEOUT;

    // record last interrupt flag
    last_interrupt_flag_ = false;
  }
}

void StateMachine::CheckLoopStatus() {
  switch (state_machine_status_) {
  case StateMachineStatus::MAIN_TASK:
    if (current_main_task_plugin_->GetTaskStatus() == TaskStatus::DONE) {
      ROS_INFO_STREAM("Task " + main_task_iterator_->task_name + " is done");

      // disable plugin control
      current_main_task_plugin_->DisableControl();
      if (current_interrupt_task_plugin_ != nullptr)
        current_interrupt_task_plugin_->DisableControl();

      // check main task vector and set loop status
      if (main_task_iterator_ == main_task_vector_.end() - 1) {
        state_machine_status_ = StateMachineStatus::END;
        ROS_INFO("State machine end");
      } else {
        state_machine_status_ = StateMachineStatus::FREE;
        ++main_task_iterator_;
      }
    }
    break;

  case StateMachineStatus::MAIN_TASK_TIMEOUT:
    ROS_INFO_STREAM("Task " + main_task_iterator_->task_name + " is timeout");

    // stop current task
    current_main_task_plugin_->StopTask();

    // disable plugin control
    current_main_task_plugin_->DisableControl();
    if (current_interrupt_task_plugin_ != nullptr)
      current_interrupt_task_plugin_->DisableControl();

    // check main task vector and set loop status
    if (main_task_iterator_ == main_task_vector_.end() - 1) {
      state_machine_status_ = StateMachineStatus::END;
      ROS_INFO("State machine end");
    } else {
      state_machine_status_ = StateMachineStatus::FREE;
      ++main_task_iterator_;
    }
    break;

  case StateMachineStatus::INTERRUPT_TASK:
    if (current_interrupt_task_plugin_->GetTaskStatus() == TaskStatus::DONE) {
      ROS_INFO_STREAM("Task " + main_task_iterator_->attach_name + " is done");

      // disable plugin control
      current_main_task_plugin_->DisableControl();
      current_interrupt_task_plugin_->DisableControl();

      // set loop status
      state_machine_status_ = StateMachineStatus::FREE;

      // reset last interrupt flag
      last_interrupt_flag_ = false;

      // compare return name and main task name
      for (auto iter = main_task_vector_.begin();
           iter < main_task_vector_.end(); iter++) {
        if (current_interrupt_task_.return_name == iter->task_name) {
          main_task_iterator_ = iter;
          break;
        }
      }
    }
    break;

  case StateMachineStatus::INTERRUPT_TASK_TIMEOUT:
    ROS_INFO_STREAM("Task " + main_task_iterator_->attach_name + " is timeout");

    // stop current task
    current_interrupt_task_plugin_->StopTask();

    // disable plugin control
    current_main_task_plugin_->DisableControl();
    current_interrupt_task_plugin_->DisableControl();

    // set loop status
    state_machine_status_ = StateMachineStatus::FREE;

    // reset last interrupt flag
    last_interrupt_flag_ = false;

    // set disable interrupt flag
    disable_interrupt_flag_ = true;
    break;

  default:
    break;
  }
}

void StateMachine::LoadPlugin(std::string &plugin_name) {
  plugin_map_[plugin_name] = plugin_loader_.createInstance(plugin_name);
  plugin_map_[plugin_name]->OnInit(mavros_pub_);
  // TODO: add blacklist and whitelist for plugins
}

void StateMachine::LoadMainTask(std::string &task_name) {
  // TODO: catch error and output if users' configuration files are wrong
  MainTask task;
  nh_.getParam(task_name + "/plugin_name", task.plugin_name);
  nh_.getParam(task_name + "/delay_timeout", task.delay_timeout);
  nh_.getParam(task_name + "/param", task.param);
  nh_.getParam(task_name + "/task_name", task.task_name);
  nh_.getParam(task_name + "/attach_name", task.attach_name);
  main_task_vector_.push_back(task);
}

void StateMachine::SetInterruptTask(std::string &task_name) {
  if (task_name != "none") {
    // TODO: catch error and output if users' configuration files are wrong
    nh_.getParam(task_name + "/plugin_name",
                 current_interrupt_task_.plugin_name);
    nh_.getParam(task_name + "/delay_timeout",
                 current_interrupt_task_.delay_timeout);
    nh_.getParam(task_name + "/param", current_interrupt_task_.param);
    nh_.getParam(task_name + "/return_name",
                 current_interrupt_task_.return_name);

    // set task
    current_interrupt_task_plugin_ =
        plugin_map_[current_interrupt_task_.plugin_name];
    current_interrupt_task_plugin_->SetTask(current_interrupt_task_.param);
    current_interrupt_task_plugin_->DisableControl();
  } else
    current_interrupt_task_plugin_ = nullptr;
}

}  // namespace whud_state_machine
