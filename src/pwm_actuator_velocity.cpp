/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-27
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_microcontroller/pwm_actuator_velocity.hpp"

namespace remote_microcontroller {

PWMActuatorVelocity::PWMActuatorVelocity(
    rclcpp::Node *node, remote_microcontroller::Implementation *microcontroller,
    int index, const std::string &prefix)
    : PWMActuator(node, microcontroller, index, prefix) {}

void PWMActuatorVelocity::velocity_set_real_(double velocity) {
  double velocity_max_mod = std::fabs(velocity_max_.as_double());
  double velocity_min_mod = std::fabs(velocity_min_.as_double());
  double velocity_mod =
      velocity_max_mod > velocity_min_mod ? velocity_max_mod : velocity_min_mod;

  double value = 0.5 + (int)(0.5 * velocity / velocity_mod);
  pwm_set(value);
}

}  // namespace remote_microcontroller
