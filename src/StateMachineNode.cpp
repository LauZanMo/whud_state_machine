/**
 * @file StateMachineNode.cpp
 * @author LauZanMo (LauZanMo@whu.edu.cn)
 * @brief State machine node
 * @version 1.0
 * @date 2021-06-04
 *
 * @copyright Copyright (c) 2021 WHU-Drones
 *
 */
#include <ros/ros.h>

#include "StateMachine.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "whud_state_machine");

  whud_state_machine::StateMachine state_machine;
  state_machine.Run();
  return 0;
}