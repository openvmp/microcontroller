/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_REMOTE_MICROCONTROLLER_INTERFACE_H
#define OPENVMP_REMOTE_MICROCONTROLLER_INTERFACE_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "remote_actuator/interface.hpp"
#include "remote_microcontroller/srv/reset.hpp"
#include "remote_serial/interface.hpp"

namespace remote_microcontroller {

class Interface {
 public:
  Interface(rclcpp::Node *node);
  virtual ~Interface() {}

 protected:
  rclcpp::Node *node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Parameter interface_prefix_;

  std::string get_prefix_();

  virtual bool reset_(bool hard, bool reflash) = 0;

 private:
  rclcpp::Service<srv::Reset>::SharedPtr srv_reset_;

  rclcpp::FutureReturnCode reset_handler_(
      const std::shared_ptr<srv::Reset::Request> request,
      std::shared_ptr<srv::Reset::Response> response);
};

}  // namespace remote_microcontroller

#endif  // OPENVMP_REMOTE_MICROCONTROLLER_INTERFACE_H
