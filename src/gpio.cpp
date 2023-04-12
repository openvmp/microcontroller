/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-27
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_microcontroller/gpio.hpp"

namespace remote_microcontroller {

GPIO::GPIO(rclcpp::Node *node,
           remote_microcontroller::Implementation *microcontroller, int channel,
           const std::string &prefix)
    : Accessory(node, microcontroller, ((uint16_t)(ADDR_GPIO_MIN + channel)),
                prefix) {}

void GPIO::gpio_set(int switch_channel, bool on) {
  // bypass Accessory::write() to control the address value
  microcontroller_->write(addr_ + switch_channel, on);
}

// GPIO does not use register read ops
void GPIO::read_cb(uint16_t value) { (void)value; }
// GPIO does not use stream ops
void GPIO::stream_cb(const std::string &value) { (void)value; }

}  // namespace remote_microcontroller
