/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_microcontroller/node.hpp"

#include "remote_microcontroller/implementation.hpp"

namespace remote_microcontroller {

Node::Node(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec)
    : rclcpp::Node::Node("remote_microcontroller") {
  ;
  intf_ = std::make_shared<Implementation>(this, exec);
}

}  // namespace remote_microcontroller
