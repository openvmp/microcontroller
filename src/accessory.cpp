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

Accessory::Accessory(rclcpp::Node* node, Implementation* microcontroller,
                     uint16_t addr, const std::string& prefix)
    : node_{node},
      microcontroller_{microcontroller},
      addr_{addr},
      initialized_{false},
      prefix_{prefix} {}

const std::string Accessory::get_prefix() const {
  std::string prefix = node_->get_namespace();

  if (prefix == "/") prefix = "";
  prefix += prefix_;

  return prefix;
}

void Accessory::init() { initialized_ = true; }

void Accessory::write(uint16_t value, bool force) {
  if (initialized_ || force) {
    microcontroller_->write(addr_, value);
  }
}

void Accessory::read() {
  if (initialized_) {
    microcontroller_->read(addr_);
  }
}

void Accessory::stream(const std::string& value) {
  if (initialized_) {
    microcontroller_->stream(addr_, value);
  }
}

}  // namespace remote_microcontroller
