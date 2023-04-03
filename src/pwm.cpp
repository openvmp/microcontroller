/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-27
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_microcontroller/pwm.hpp"

namespace remote_microcontroller {

PWM::PWM(rclcpp::Node *node,
         remote_microcontroller::Implementation *microcontroller, int index,
         const std::string &prefix)
    : Accessory(microcontroller, ((uint16_t)(ADDR_PWM_MIN + index)), prefix) {
  node->declare_parameter("pwm_min", 0);
  node->get_parameter("pwm_min", param_min_);
  node->declare_parameter("pwm_max", 255);
  node->get_parameter("pwm_max", param_max_);

  topic_pwm_ =
      node->create_publisher<std_msgs::msg::Float64>(prefix + "/pwm", 1);
}

void PWM::pwm_set(double value) {
  double min = param_min_.as_double();
  double max = param_max_.as_double();

  int val = value * (int)(max - min) + (int)min;
  if (val > max) val = max;
  if (val < min) val = min;
  write(val);
}

void PWM::pwm_set_callback_(std_msgs::msg::Float64 value) {
  pwm_set(value.data);
}

// PWM does not use register read/write ops
void PWM::read_cb(uint16_t value) { (void)value; }
// PWM does not use stream ops
void PWM::stream_cb(const std::string &value) { (void)value; }

}  // namespace remote_microcontroller
