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
  WhudBasicControl() : PluginBase(), sm_nh_("~basic_control"){};

  enum Command { NONE = 0, TAKEOFF, LAND, HEIGHT_CONTROL, YAW_CONTROL };
  void OnInit(MavRosPublisher &mavros_pub) {
    mavros_pub_ = &mavros_pub;
  }

  bool SetTask(ros::V_string param) {
    PluginBase::SetTask(param);
    if (param[0] == "takeoff") {
      takeoff.data.clear();
      // set z axis speed
      takeoff.data.push_back(atof(param[1].c_str()));

      // set height
      takeoff.data.push_back(atof(param[2].c_str()));

      command_ = Command::TAKEOFF;

    } else if (param[0] == "land") {
      // set z axis speed
      land.data = atof(param[1].c_str());

      command_ = Command::LAND;

    } else if (param[0] == "height_control") {
      height_control.data.clear();
      // set z axis speed
      height_control.data.push_back(atof(param[1].c_str()));

      // set target height(relatively)
      height_control.data.push_back(atof(param[2].c_str()));

      command_ = Command::HEIGHT_CONTROL;

    } else if (param[0] == "yaw_control") {
      yaw_control.data.clear();
      // set target angle
      yaw_control.data.push_back(atof(param[1].c_str()));

      // 0: absolute, 1: relative
      yaw_control.data.push_back(atof(param[2].c_str()));

      command_ = Command::YAW_CONTROL;

    } else {
      command_ = Command::NONE;
    }
  }

  virtual void TaskSpin() override {
    sm_nh_.getParam("/mavros/whud_basic/ack_cmd_index", mavros_command_);
    sm_nh_.getParam("/mavros/whud_basic/ack_result", mavros_result_);

    switch (command_) {
    case Command::TAKEOFF:
      if (mavros_command_ == 24 && mavros_result_ == 0) {
        task_status_ = TaskStatus::DONE;
      } else if (mavros_command_ != 24 && mavros_result_ != 5) {
        if (mavros_pub_ != nullptr && control_flag_ == true) {
          mavros_pub_->takeoff_pub.publish(takeoff);
        }
      }
      break;
    case Command::LAND:
      if (mavros_command_ == 23 && mavros_result_ == 0) {
        task_status_ = TaskStatus::DONE;
      } else if (mavros_command_ != 23 && mavros_result_ != 5) {
        if (mavros_pub_ != nullptr && control_flag_ == true) {
          mavros_pub_->land_pub.publish(land);
        }
      }
      break;
    case Command::HEIGHT_CONTROL:
      if (mavros_command_ == 113 && mavros_result_ == 0) {
        task_status_ = TaskStatus::DONE;
      } else if (mavros_command_ != 113 && mavros_result_ != 5) {
        if (mavros_pub_ != nullptr && control_flag_ == true) {
          mavros_pub_->height_pub.publish(height_control);
        }
      }
      break;
    case Command::YAW_CONTROL:
      if (mavros_command_ != 115 && mavros_result_ == 0) {
        task_status_ = TaskStatus::DONE;
      } else if (mavros_command_ == 115 && mavros_result_ != 5) {
        if (mavros_pub_ != nullptr && control_flag_ == true) {
          mavros_pub_->yaw_pub.publish(yaw_control);
        }
      }
      break;
    default:
      break;
    }
  }

  virtual void StopTask() override {}

private:
  ros::NodeHandle sm_nh_;
  std_msgs::Float64MultiArray takeoff;
  float takeoff_params[2];
  std_msgs::Float64 land;
  std_msgs::Float64MultiArray height_control;
  float height_control_params[2];
  std_msgs::Float64MultiArray yaw_control;
  float yaw_control_params[2];
  Command command_;

  int mavros_command_;
  int mavros_result_;
};

}  // namespace whud_state_machine

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(whud_state_machine::WhudBasicControl,
                       whud_state_machine::PluginBase)
