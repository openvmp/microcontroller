/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-04-09
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_REMOTE_MICROCONTROLLER_PULDIR_STEPPER_DRIVER_H
#define OPENVMP_REMOTE_MICROCONTROLLER_PULDIR_STEPPER_DRIVER_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "remote_microcontroller/puldir.hpp"
#include "remote_stepper_driver/implementation.hpp"
#include "remote_stepper_driver/srv/param_ppr_get.hpp"
#include "remote_stepper_driver/srv/param_ppr_set.hpp"

namespace remote_microcontroller {

class PulDirStepperDriver : public PulDir,
                            public remote_stepper_driver::Implementation {
 public:
  PulDirStepperDriver(rclcpp::Node *node,
                      remote_microcontroller::Implementation *microcontroller,
                      int pul_channel, int dir_channel,
                      const std::string &prefix);
  rclcpp::Parameter param_ppr_;

  virtual bool has_position() override { return false; }
  virtual bool has_velocity() override { return true; }

 protected:
  virtual void position_set_real_(double) override {}
  virtual void velocity_set_real_(double) override;

  virtual rclcpp::FutureReturnCode param_ppr_get_handler_(
      const std::shared_ptr<remote_stepper_driver::srv::ParamPprGet::Request>
          request,
      std::shared_ptr<remote_stepper_driver::srv::ParamPprGet::Response>
          response) override;
  virtual rclcpp::FutureReturnCode param_ppr_set_handler_(
      const std::shared_ptr<remote_stepper_driver::srv::ParamPprSet::Request>
          request,
      std::shared_ptr<remote_stepper_driver::srv::ParamPprSet::Response>
          response) override;

 private:
  uint32_t ppr_;
};

}  // namespace remote_microcontroller

#endif  // OPENVMP_REMOTE_MICROCONTROLLER_PULDIR_STEPPER_DRIVER_H
