/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-27
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_microcontroller/pwm_actuator.hpp"

#include <cmath>

namespace remote_microcontroller {

PWMActuator::PWMActuator(
    rclcpp::Node *node, remote_microcontroller::Implementation *microcontroller,
    int index, const std::string &prefix)
    : PWM(microcontroller, index, prefix),
      remote_actuator::Implementation(node, prefix) {
  auto position_max = position_max_.as_double();
  auto position_min = position_min_.as_double();
  position_max_mod_ = std::fabs(position_max) > std::fabs(position_min)
                          ? std::fabs(position_max)
                          : std::fabs(position_min);

  auto velocity_max = velocity_max_.as_double();
  auto velocity_min = velocity_min_.as_double();
  velocity_max_mod_ = std::fabs(velocity_max) > std::fabs(velocity_min)
                          ? std::fabs(velocity_max)
                          : std::fabs(velocity_min);
}

}  // namespace remote_microcontroller
