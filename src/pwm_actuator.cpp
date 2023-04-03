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
    : PWM(node, microcontroller, index, prefix),
      remote_actuator::Implementation(node, prefix) {}

}  // namespace remote_microcontroller
