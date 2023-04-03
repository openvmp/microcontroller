/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-27
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_microcontroller/accessory.hpp"

namespace remote_microcontroller {

Accessory::Accessory(rclcpp::Node *node, Implementation* microcontroller, uint16_t addr,
                     const std::string& prefix)
    : node_{node}, microcontroller_{microcontroller}, addr_{addr},
      prefix_{prefix} {}

const std::string Accessory::get_prefix() const {
  std::string prefix = node_->get_namespace();

  if (prefix == "/") prefix = "";
  prefix += prefix_;

  return prefix;
}

void Accessory::write(uint16_t value) { microcontroller_->write(addr_, value); }

void Accessory::stream(const std::string& value) {
  microcontroller_->stream(addr_, value);
}

}  // namespace remote_microcontroller
