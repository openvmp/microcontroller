/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-27
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_microcontroller/puldir.hpp"

#include "remote_microcontroller/proto_gpio.hpp"

namespace remote_microcontroller {

PulDir::PulDir(rclcpp::Node *node,
               remote_microcontroller::Implementation *microcontroller,
               int pul_channel, int dir_channel, const std::string &prefix)
    : Accessory(node, microcontroller, ((uint16_t)(ADDR_PUL_MIN + pul_channel)),
                prefix),
      dir_channel_{dir_channel} {
  topic_pps_ = node->create_subscription<std_msgs::msg::Int32>(
      get_prefix() + "/pps", 1,
      std::bind(&PulDir::puldir_set_callback_, this, std::placeholders::_1));
}

void PulDir::puldir_set(int dir, uint16_t pps) {
  if (last_dir_ != dir && dir_channel_ != -1) {
    if (last_pps_ != 0) {
      // set velocity to 0 before switching the direction
      write(0);
      last_pps_ = 0;
    }

    microcontroller_->write(ADDR_GPIO_MIN + dir_channel_, dir);
    last_dir_ = dir;
  }

  if (last_pps_ != pps) {
    write(pps);
    last_pps_ = pps;
  }
}

void PulDir::puldir_set_callback_(const std_msgs::msg::Int32::SharedPtr value) {
  int dir = value->data >= 0 ? PULDIR_CW : PULDIR_CCW;
  uint32_t pps = ::abs(value->data);
  puldir_set(dir, pps);
}

// PulDir does not use register read ops
void PulDir::read_cb(uint16_t value) { (void)value; }
// PulDir does not use stream ops
void PulDir::stream_cb(const std::string &value) { (void)value; }

}  // namespace remote_microcontroller
