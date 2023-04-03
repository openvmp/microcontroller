/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-27
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_microcontroller/pwm_actuator_position.hpp"

namespace remote_microcontroller {

PWMActuatorPosition::PWMActuatorPosition(
    rclcpp::Node *node, remote_microcontroller::Implementation *microcontroller,
    int index, const std::string &prefix)
    : PWMActuator(node, microcontroller, index, prefix) {}

void PWMActuatorPosition::position_set_real_(double position) {
  double position_max_mod = std::fabs(position_max_.as_double());
  double position_min_mod = std::fabs(position_min_.as_double());
  double position_mod =
      position_max_mod > position_min_mod ? position_max_mod : position_min_mod;

  double value = 0.5 + (int)(0.5 * position / position_mod);
  pwm_set(value);
}

}  // namespace remote_microcontroller
