/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_REMOTE_MICROCONTROLLER_IMPLEMENTATION_H
#define OPENVMP_REMOTE_MICROCONTROLLER_IMPLEMENTATION_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "remote_microcontroller/interface.hpp"
#include "remote_microcontroller/srv/reset.hpp"
#include "ros2_serial/implementation.hpp"

#define MICROCONTROLLER_SERVICE_RESET "/reset"

namespace remote_microcontroller {

class Accessory;

class Implementation final : public Interface {
 public:
  Implementation(
      rclcpp::Node *node,
      std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec);
  virtual ~Implementation() {}

  void write(uint16_t addr, uint16_t value);
  void read(uint16_t addr);
  void stream(uint16_t addr, const std::string &value);

 private:
  bool initialized_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec_;
  std::shared_ptr<ros2_serial::Interface> prov_;
  std::string input_queue_;
  std::mutex input_queue_mutex_;

  rclcpp::Parameter param_config_;

  std::map<uint16_t, std::shared_ptr<Accessory>> accessories_;

  rclcpp::Service<srv::Reset>::SharedPtr srv_reset_;
  rclcpp::FutureReturnCode reset_handler_(
      const std::shared_ptr<srv::Reset::Request> request,
      std::shared_ptr<srv::Reset::Response> response);
  virtual bool reset_(bool hard, bool reflash) override;

  // input_cb is a static method to receive callbacks from the serial module.
  static void input_cb_(const std::string &msg, void *user_data);
  // input_cb_real_ is the real handler of the callbacks from the serial module.
  void input_cb_real_(const std::string &msg);
};

}  // namespace remote_microcontroller

#endif  // OPENVMP_REMOTE_MICROCONTROLLER_IMPLEMENTATION_H
