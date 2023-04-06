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
  std::lock_guard<std::mutex> guard(param_maxmin_lock_);

  position -= position_min_;
  position /= position_max_ - position_min_;
  // position is now remaped from [min;max] to [0;1]

  pwm_set(pwm_min_ + position * (pwm_max_ - pwm_min_));
}

}  // namespace remote_microcontroller
