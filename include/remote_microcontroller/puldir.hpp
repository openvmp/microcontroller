/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-04-09
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_REMOTE_MICROCONTROLLER_PULDIR_H
#define OPENVMP_REMOTE_MICROCONTROLLER_PULDIR_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "remote_microcontroller/accessory.hpp"
#include "remote_microcontroller/proto_pul.hpp"
#include "std_msgs/msg/int32.hpp"

namespace remote_microcontroller {

class PulDir : public Accessory {
 public:
  static const int PULDIR_CW = 0, PULDIR_CCW = 1;

  PulDir(rclcpp::Node *node,
         remote_microcontroller::Implementation *microcontroller,
         int pul_channel, int dir_channel, const std::string &prefix);

 protected:
  void puldir_set(int dir, uint16_t ppr);

  // implementation of Accessory
  virtual void read_cb(uint16_t value) override;
  virtual void stream_cb(const std::string &value) override;

 private:
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr topic_pps_;
  int dir_channel_;
  int last_dir_;
  uint16_t last_pps_;

  void puldir_set_callback_(const std_msgs::msg::Int32::SharedPtr);
};

}  // namespace remote_microcontroller

#endif  // OPENVMP_REMOTE_MICROCONTROLLER_PULDIR_H
