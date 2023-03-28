/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "rclcpp/rclcpp.hpp"
#include "remote_microcontroller/node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto node = std::make_shared<remote_microcontroller::Node>(exec);

  exec->add_node(node);
  exec->spin();

  rclcpp::shutdown();
  return 0;
}
