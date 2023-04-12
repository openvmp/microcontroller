/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-04-09
 *
 * Licensed under Apache License, Version 2.0.
 */
#ifndef OPENVMP_REMOTE_MICROCONTROLLER_GPIO_H
#define OPENVMP_REMOTE_MICROCONTROLLER_GPIO_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "remote_microcontroller/accessory.hpp"
#include "remote_microcontroller/proto_gpio.hpp"

namespace remote_microcontroller {

class GPIO : public Accessory {
 public:
  GPIO(rclcpp::Node *node,
       remote_microcontroller::Implementation *microcontroller, int channel,
       const std::string &prefix);

 protected:
  // Set pwm using values from min to max (e.g. from 0 to 180)
  void gpio_set(int switch_channel, bool on);

  // implementation of Accessory
  virtual void read_cb(uint16_t value) override;
  virtual void stream_cb(const std::string &value) override;
};

}  // namespace remote_microcontroller

#endif  // OPENVMP_REMOTE_MICROCONTROLLER_GPIO_H
