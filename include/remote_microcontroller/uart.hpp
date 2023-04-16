/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-27
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_REMOTE_MICROCONTROLLER_UART_H
#define OPENVMP_REMOTE_MICROCONTROLLER_UART_H

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "remote_microcontroller/accessory.hpp"
#include "remote_microcontroller/implementation.hpp"
#include "remote_microcontroller/proto_uart.hpp"
#include "ros2_serial/implementation.hpp"

namespace remote_microcontroller {

class UART : public Accessory, public ros2_serial::Implementation {
 public:
  UART(rclcpp::Node *node,
       remote_microcontroller::Implementation *microcontroller, int index,
       const std::string &prefix);
  rclcpp::Parameter baud_rate;

 protected:
  // implementation of Accessory
  virtual void init() override;
  virtual void read_cb(uint16_t value) override;
  virtual void stream_cb(const std::string &value) override;

  // implementation of ros2_serial::Implementation
  virtual void output(const std::string &msg) override;
  virtual void register_input_cb(void (*input_cb)(const std::string &msg,
                                                  void *user_data),
                                 void *user_data) override;
  virtual void inject_input(const std::string &msg) override;

 private:
  // callbacks
  void (*volatile input_cb_)(const std::string &msg, void *user_data);
  void *volatile input_cb_user_data_;
  std::mutex input_cb_mutex_;
};

}  // namespace remote_microcontroller

#endif  // OPENVMP_REMOTE_MICROCONTROLLER_UART_H
