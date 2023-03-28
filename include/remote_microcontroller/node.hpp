/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_REMOTE_MICROCONTROLLER_NODE_H
#define OPENVMP_REMOTE_MICROCONTROLLER_NODE_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "remote_microcontroller/interface.hpp"

namespace remote_microcontroller {

class Node : public rclcpp::Node {
 public:
  Node(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec);

 private:
  std::shared_ptr<Interface> intf_;
};

}  // namespace remote_microcontroller

#endif  // OPENVMP_REMOTE_MICROCONTROLLER_NODE_H
