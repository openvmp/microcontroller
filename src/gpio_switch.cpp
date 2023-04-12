/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-04-09
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_microcontroller/gpio_switch.hpp"

namespace remote_microcontroller {

GPIOSwitch::GPIOSwitch(rclcpp::Node *node,
                       remote_microcontroller::Implementation *microcontroller,
                       int index, const std::string &prefix)
    : GPIO(node, microcontroller, index, prefix),
      remote_switch::Implementation(node) {}

void GPIOSwitch::switch_handler_real_(
    const std::shared_ptr<remote_switch::srv::Switch::Request> request,
    std::shared_ptr<remote_switch::srv::Switch::Response> response) {
  (void)response;
  gpio_set(request->channel, request->on);
}

}  // namespace remote_microcontroller