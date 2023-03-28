/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_microcontroller/interface.hpp"

#include "remote_actuator/factory.hpp"
#include "ros2_serial/factory.hpp"

namespace remote_microcontroller {

Interface::Interface(rclcpp::Node *node) : node_{node} {
  callback_group_ =
      node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  node->declare_parameter("microcontroller_prefix",
                          "/microcontroller/" + std::string(node_->get_name()));
  node->get_parameter("microcontroller_prefix", interface_prefix_);

  auto prefix = get_prefix_();
}

std::string Interface::get_prefix_() {
  std::string prefix = std::string(node_->get_namespace());
  if (prefix.length() > 0 && prefix[prefix.length() - 1] == '/') {
    prefix = prefix.substr(0, prefix.length() - 1);
  }
  prefix += interface_prefix_.as_string();
  return prefix;
}

}  // namespace remote_microcontroller
