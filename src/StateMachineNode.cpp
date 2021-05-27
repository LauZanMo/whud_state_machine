#include <ros/ros.h>

#include "StateMachine.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "whud_state_machine");

  whud_state_machine::StateMachine state_machine;
  state_machine.Run();
  return 0;
}