/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-27
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_REMOTE_MICROCONTROLLER_PWM_ACTUATOR_VELOCITY_H
#define OPENVMP_REMOTE_MICROCONTROLLER_PWM_ACTUATOR_VELOCITY_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "remote_microcontroller/pwm_actuator.hpp"

namespace remote_microcontroller {

class PWMActuatorVelocity : public PWMActuator {
 public:
  PWMActuatorVelocity(rclcpp::Node *node,
                      remote_microcontroller::Implementation *microcontroller,
                      int index, const std::string &prefix);

  virtual bool has_velocity() override { return true; }

 protected:
  virtual void position_set_real_(double) override {}
  virtual void velocity_set_real_(double) override;
};

}  // namespace remote_microcontroller

#endif  // OPENVMP_REMOTE_MICROCONTROLLER_PWM_ACTUATOR_VELOCITY_H
