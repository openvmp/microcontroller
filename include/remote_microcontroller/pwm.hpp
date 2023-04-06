/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-27
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_REMOTE_MICROCONTROLLER_PWM_H
#define OPENVMP_REMOTE_MICROCONTROLLER_PWM_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "remote_microcontroller/accessory.hpp"
#include "remote_microcontroller/proto_pwm.hpp"
#include "std_msgs/msg/u_int16.hpp"

namespace remote_microcontroller {

class PWM : public Accessory {
 public:
  PWM(rclcpp::Node *node,
      remote_microcontroller::Implementation *microcontroller, int index,
      const std::string &prefix);

 protected:
  uint16_t pwm_min_, pwm_max_;

  // Set pwm using values from min to max (e.g. from 0 to 180)
  void pwm_set(uint16_t value);

  // implementation of Accessory
  virtual void read_cb(uint16_t value) override;
  virtual void stream_cb(const std::string &value) override;

 private:
  rclcpp::Parameter param_min_, param_max_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr topic_pwm_min_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr topic_pwm_max_;
  rclcpp::Subscription<std_msgs::msg::UInt16>::SharedPtr topic_pwm_;

  void pwm_set_callback_(std_msgs::msg::UInt16);
};

}  // namespace remote_microcontroller

#endif  // OPENVMP_REMOTE_MICROCONTROLLER_PWM_H
