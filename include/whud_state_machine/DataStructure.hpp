/**
 * @file DataStructure.hpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @brief State machine data structure
 * @version 1.0
 * @date 2021-06-04
 *
 * @copyright Copyright (c) 2021 WHU-Drones
 *
 */
#pragma once

#include <ros/ros.h>

#include <iostream>

namespace whud_state_machine {

enum StateMachineStatus {
  FREE = 0,
  MAIN_TASK,
  INTERRUPT_TASK,
  MAIN_TASK_TIMEOUT,
  INTERRUPT_TASK_TIMEOUT,
  END
};

enum TaskStatus { RUN = 0, DONE };

struct MainTask {
  std::string plugin_name;
  int delay_timeout;
  ros::V_string param;
  std::string task_name;
  std::string attach_name;
};

struct InterruptTask {
  std::string plugin_name;
  int delay_timeout;
  ros::V_string param;
  std::string return_name;
};

struct MavRosPublisher {
  // basic cmd
  ros::Publisher takeoff_pub;
  ros::Publisher land_pub;
  ros::Publisher height_pub;
  ros::Publisher yaw_pub;

  // nav cmd
  ros::Publisher cmd_vel_pub;
  ros::Publisher conversion_pub;
};

}  // namespace whud_state_machine
