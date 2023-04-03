/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-27
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_REMOTE_MICROCONTROLLER_PWM_ACTUATOR_H
#define OPENVMP_REMOTE_MICROCONTROLLER_PWM_ACTUATOR_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "remote_actuator/implementation.hpp"
#include "remote_microcontroller/pwm.hpp"

namespace remote_microcontroller {

class PWMActuator : public PWM, public remote_actuator::Implementation {
 public:
  PWMActuator(rclcpp::Node *node,
              remote_microcontroller::Implementation *microcontroller,
              int index, const std::string &prefix);
};

}  // namespace remote_microcontroller

#endif  // OPENVMP_REMOTE_MICROCONTROLLER_PWM_ACTUATOR_H
