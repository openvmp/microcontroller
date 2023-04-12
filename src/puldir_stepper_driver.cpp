/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-04-09
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_microcontroller/puldir_stepper_driver.hpp"

namespace remote_microcontroller {

PulDirStepperDriver::PulDirStepperDriver(
    rclcpp::Node *node, remote_microcontroller::Implementation *microcontroller,
    int pul_channel, int dir_channel, const std::string &prefix)
    : PulDir(node, microcontroller, pul_channel, dir_channel, prefix),
      remote_stepper_driver::Implementation(node) {
  node->declare_parameter("ppr", 0);
  node->get_parameter("ppr", param_ppr_);
  ppr_ = param_ppr_.as_int();
}

void PulDirStepperDriver::velocity_set_real_(double velocity) {
  std::lock_guard<std::mutex> guard(param_maxmin_lock_);

  int dir;
  if (velocity >= 0) {
    dir = PULDIR_CW;
  } else {
    dir = PULDIR_CCW;
  }

  uint32_t pps = ppr_ * velocity / M_PI;
  if (pps > 65535) {
    pps = 65535;
  }

  puldir_set(dir, pps);
}

rclcpp::FutureReturnCode PulDirStepperDriver::param_ppr_get_handler_(
    const std::shared_ptr<remote_stepper_driver::srv::ParamPprGet::Request>
        request,
    std::shared_ptr<remote_stepper_driver::srv::ParamPprGet::Response>
        response) {
  (void)request;
  response->ppr = ppr_;
  return rclcpp::FutureReturnCode::SUCCESS;
}
rclcpp::FutureReturnCode PulDirStepperDriver::param_ppr_set_handler_(
    const std::shared_ptr<remote_stepper_driver::srv::ParamPprSet::Request>
        request,
    std::shared_ptr<remote_stepper_driver::srv::ParamPprSet::Response>
        response) {
  (void)response;
  ppr_ = request->ppr;
  return rclcpp::FutureReturnCode::SUCCESS;
}

}  // namespace remote_microcontroller
