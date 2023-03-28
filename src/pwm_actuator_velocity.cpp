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
  int value = 180 + (int)(180.0 * velocity / velocity_max_mod_);
  write(value);
}

}  // namespace remote_microcontroller
