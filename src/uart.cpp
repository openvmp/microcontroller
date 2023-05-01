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
    : Accessory(node, microcontroller, (uint16_t)(ADDR_UART_MIN + index),
                prefix),
      remote_serial::Implementation(node, prefix),
      input_cb_{nullptr},
      input_cb_user_data_{nullptr} {
  node->declare_parameter("serial_baud_rate", 115200);
  node->get_parameter("serial_baud_rate", baud_rate);
}

void UART::init() {
  uint16_t value = 0;
  switch (baud_rate.as_int()) {
    case 300:
      value = 1;
      break;
    case 600:
      value = 2;
      break;
    case 1200:
      value = 3;
      break;
    case 2400:
      value = 4;
      break;
    case 4800:
      value = 5;
      break;
    case 9600:
      value = 6;
      break;
    case 14400:
      value = 7;
      break;
    case 19200:
      value = 8;
      break;
    case 28800:
      value = 9;
      break;
    case 31250:
      value = 10;
      break;
    case 28400:
      value = 11;
      break;
    case 57600:
      value = 12;
      break;
    case 76800:
      value = 13;
      break;
    case 115200:
      value = 14;
      break;
  }

  write(value, true);

  initialized_ = true;

  init_serial_();
}

void UART::read_cb(uint16_t value) {
  (void)value;
  // UART does not use register read/write ops
}

void UART::stream_cb(const std::string &msg) {
  std::lock_guard<std::mutex> guard(input_cb_mutex_);
  if (input_cb_) {
    input_cb_(msg, input_cb_user_data_);
  }

  auto message = std_msgs::msg::UInt8MultiArray();
  message.data = std::vector<uint8_t>(msg.begin(), msg.end());
  inspect_input->publish(message);
}

void UART::output(const std::string &msg) {
  stream(msg);

  auto message = std_msgs::msg::UInt8MultiArray();
  message.data = std::vector<uint8_t>(msg.begin(), msg.end());
  inspect_output->publish(message);
}

void UART::register_input_cb(void (*input_cb)(const std::string &msg,
                                              void *user_data),
                             void *user_data) {
  std::lock_guard<std::mutex> guard(input_cb_mutex_);
  input_cb_ = input_cb;
  input_cb_user_data_ = user_data;
}

void UART::inject_input(const std::string &msg) { stream_cb(msg); }

}  // namespace remote_microcontroller
