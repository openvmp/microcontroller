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
  std::lock_guard<std::mutex> guard(param_maxmin_lock_);

  double value = 0.5 + (int)(0.5 * velocity / velocity_mod_);
  pwm_set(value);
}

}  // namespace remote_microcontroller
