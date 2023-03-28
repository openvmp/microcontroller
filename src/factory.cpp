/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_microcontroller/factory.hpp"

#include "remote_microcontroller/implementation.hpp"

namespace remote_microcontroller {

std::shared_ptr<Interface> Factory::New(
    rclcpp::Node* node,
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec) {
  return std::make_shared<Implementation>(node, exec);
}

}  // namespace remote_microcontroller
