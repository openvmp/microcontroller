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
    : Accessory(node, microcontroller, ((uint16_t)(ADDR_PWM_MIN + index)),
                prefix) {
  node->declare_parameter("pwm_min", 0);
  node->get_parameter("pwm_min", param_min_);
  node->declare_parameter("pwm_max", 255);
  node->get_parameter("pwm_max", param_max_);

  // TODO(clairbee): create a parameter event handler to track parameter changes
  pwm_min_ = param_min_.as_int();
  pwm_max_ = param_max_.as_int();

  // Publish the min/max values so that the consumers now the boundaries
  topic_pwm_min_ = node->create_publisher<std_msgs::msg::UInt16>(
      get_prefix() + "/pwm_min", 1);
  topic_pwm_min_->publish(std_msgs::msg::UInt16().set__data(pwm_min_));
  topic_pwm_max_ = node->create_publisher<std_msgs::msg::UInt16>(
      get_prefix() + "/pwm_max", 1);
  topic_pwm_max_->publish(std_msgs::msg::UInt16().set__data(pwm_max_));

  topic_pwm_ = node->create_subscription<std_msgs::msg::UInt16>(
      get_prefix() + "/pwm", 1,
      std::bind(&PWM::pwm_set_callback_, this, std::placeholders::_1));
}

void PWM::pwm_set(uint16_t value) {
  if (value > pwm_max_) value = pwm_max_;
  if (value < pwm_min_) value = pwm_min_;
  write(value);
}

void PWM::pwm_set_callback_(const std_msgs::msg::UInt16::SharedPtr value) {
  pwm_set(value->data);
}

// PWM does not use register read/write ops
void PWM::read_cb(uint16_t value) { (void)value; }
// PWM does not use stream ops
void PWM::stream_cb(const std::string &value) { (void)value; }

}  // namespace remote_microcontroller
