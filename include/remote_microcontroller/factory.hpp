/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_REMOTE_MICROCONTROLLER_FACTORY_H
#define OPENVMP_REMOTE_MICROCONTROLLER_FACTORY_H

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "remote_microcontroller/interface.hpp"

namespace remote_microcontroller {

class Factory {
 public:
  static std::shared_ptr<Interface> New(
      rclcpp::Node *node,
      std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec);
};

}  // namespace remote_microcontroller

#endif  // OPENVMP_REMOTE_MICROCONTROLLER_FACTORY_H
