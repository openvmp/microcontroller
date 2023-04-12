/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-04-09
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_REMOTE_MICROCONTROLLER_GPIO_SWITCH_H
#define OPENVMP_REMOTE_MICROCONTROLLER_GPIO_SWITCH_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "remote_microcontroller/gpio.hpp"
#include "remote_switch/implementation.hpp"

namespace remote_microcontroller {

class GPIOSwitch : public GPIO, public remote_switch::Implementation {
 public:
  GPIOSwitch(rclcpp::Node *node,
             remote_microcontroller::Implementation *microcontroller,
             int channel, const std::string &prefix);

  virtual void switch_handler_real_(
      const std::shared_ptr<remote_switch::srv::Switch::Request> request,
      std::shared_ptr<remote_switch::srv::Switch::Response> response) override;
};

}  // namespace remote_microcontroller

#endif  // OPENVMP_REMOTE_MICROCONTROLLER_GPIO_SWITCH_H
