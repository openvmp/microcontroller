/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-27
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_microcontroller/uart.hpp"

namespace remote_microcontroller {

UART::UART(rclcpp::Node *node,
           remote_microcontroller::Implementation *microcontroller, int index,
           const std::string &prefix)
    : Accessory(microcontroller, (uint16_t)(ADDR_UART_MIN + index), prefix),
      ros2_serial::Implementation(node, prefix) {}

void UART::read_cb(uint16_t value) {
  (void)value;
  // UART does not use register read/write ops
}
void UART::stream_cb(const std::string &value) { inject_input(value); }

void UART::output(const std::string &msg) { stream(msg); }

void UART::register_input_cb(void (*input_cb)(const std::string &msg,
                                              void *user_data),
                             void *user_data) {
  std::lock_guard<std::mutex> guard(input_cb_mutex_);
  input_cb_ = input_cb;
  input_cb_user_data_ = user_data;
}

void UART::inject_input(const std::string &msg) {
  std::lock_guard<std::mutex> guard(input_cb_mutex_);
  if (input_cb_) {
    input_cb_(msg, input_cb_user_data_);
  }
}

}  // namespace remote_microcontroller
