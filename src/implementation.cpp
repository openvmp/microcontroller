/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_microcontroller/implementation.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <locale>

#include "remote_microcontroller/config.hpp"
#include "remote_microcontroller/interface.hpp"
#include "remote_microcontroller/proto_mgmt.hpp"
#include "remote_microcontroller/proto_service.hpp"
#include "remote_microcontroller/pwm_actuator_position.hpp"
#include "remote_microcontroller/pwm_actuator_velocity.hpp"
#include "remote_microcontroller/uart.hpp"
#include "ros2_serial/factory.hpp"
#include "ros2_serial/utils.hpp"
#include "yaml-cpp/yaml.h"

namespace remote_microcontroller {

Implementation::Implementation(
    rclcpp::Node *node,
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec)
    : Interface{node}, exec_{exec} {
  auto prefix = get_prefix_();

  prov_ = ros2_serial::Factory::New(node);
  prov_->register_input_cb(&Implementation::input_cb_, this);

  srv_reset_ = node_->create_service<srv::Reset>(
      prefix + MICROCONTROLLER_SERVICE_RESET,
      std::bind(&Implementation::reset_handler_, this, std::placeholders::_1,
                std::placeholders::_2),
      ::rmw_qos_profile_default, callback_group_);

  node->declare_parameter("microcontroller_config",
                          "config/microcontroller.yaml");
  node->get_parameter("microcontroller_config", param_config_);

  // Static part is done, now dynamic behavior driven by the config
  auto config_filename = param_config_.as_string();
  YAML::Node config = YAML::LoadFile(config_filename);

  if (!config || config.IsNull() || !config.IsMap()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to parse: %s",
                 config_filename.c_str());
    return;
  }

  auto pwms = config[MICROCONTROLLER_CONFIG_CHAPTER_PWM];
  int index = 0;
  for (const auto &pwm : pwms) {
    RCLCPP_DEBUG(node_->get_logger(), "Found a pwm entry in the config");
    auto type = pwm["type"].as<std::string>();
    auto node_name = "driver_microcontroller_" + pwm["name"].as<std::string>();
    auto node_prefix = pwm["prefix"].as<std::string>();

    int channel = index;
    if (pwm["channel"]) {
      channel = pwm["channel"].as<int>();
    }

    if (type == "actuator_position" || type == "actuator_velocity" ||
        type == "simple_pwm") {
      std::string ns = node_->get_namespace();
      auto node_options = rclcpp::NodeOptions{};

      // Prepare node parameters
      for (const auto &param : pwm) {
        const auto &key = param.first.as<std::string>();

        if (key == "type" || key == "name" || key == "prefix") {
          continue;
        }

        try {
          auto value = param.second.as<int>();
          node_options.parameter_overrides().push_back({key, value});
        } catch (const std::exception &_e) {
          try {
            auto value = param.second.as<double>();
            node_options.parameter_overrides().push_back({key, value});
          } catch (const std::exception &_e) {
            auto value = param.second.as<std::string>();
            node_options.parameter_overrides().push_back({key, value});
          }
        }
      }

      // Create the node
      node_options.use_intra_process_comms(true);
      auto pwm_node =
          std::make_shared<rclcpp::Node>(node_name, ns, node_options);
      exec->add_node(pwm_node);

      // Instantiate the accessory
      std::shared_ptr<PWM> ptr;
      if (type == "actuator_position") {
        RCLCPP_DEBUG(node_->get_logger(), "Found a position actuator entry");
        ptr = std::make_shared<PWMActuatorVelocity>(pwm_node.get(), this,
                                                    channel, node_prefix);
      } else if (type == "actuator_position") {
        RCLCPP_DEBUG(node_->get_logger(), "Found a velocity actuator entry");
        ptr = std::make_shared<PWMActuatorPosition>(pwm_node.get(), this,
                                                    channel, node_prefix);
      } else {
        RCLCPP_DEBUG(node_->get_logger(), "Found a simple pwm entry");
        ptr = std::make_shared<PWM>(pwm_node.get(), this, channel, node_prefix);
      }

      // Store the accessory in the collection
      accessories_.insert({ptr->get_addr(), ptr});
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Found an incorrect pwm entry");
    }
    index++;
  }

  auto uarts = config[MICROCONTROLLER_CONFIG_CHAPTER_UART];
  index = 0;
  for (auto uart : uarts) {
    RCLCPP_DEBUG(node_->get_logger(), "Found an uart entry in the config");
    if (uart["serial"]) {
      RCLCPP_DEBUG(node_->get_logger(), "Found a serial entry");

      int channel = index;
      if (uart["channel"]) {
        channel = uart["channel"].as<int>();
      }

      // Create the node
      auto node_name = "driver_microcontroller_uart" + std::to_string(index);
      std::string ns = node_->get_namespace();
      auto node_options = rclcpp::NodeOptions{};
      node_options.use_intra_process_comms(true);
      auto uart_node =
          std::make_shared<rclcpp::Node>(node_name, ns, node_options);
      exec->add_node(uart_node);

      // Instantiate the accessory
      auto ptr = std::make_shared<UART>(uart_node.get(), this, channel,
                                        uart["serial"].as<std::string>());

      // Store the accessory in the collection
      accessories_.insert({ptr->get_addr(), ptr});
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Found an incorrect uart entry");
    }
    index++;
  }
}

bool Implementation::reset_(bool hard, bool reflash) {
  (void)hard;
  (void)reflash;
  // TODO(clairbee): implement reset
  return false;
}

void Implementation::write(uint16_t addr, uint16_t value) {
  uint8_t cmd[7];
  rm_mgmt_pack7_write(addr, value, cmd);
  prov_->output(std::string((char *)&cmd[0], sizeof(cmd)));
}

void Implementation::stream(uint16_t addr, const std::string &value) {
  uint8_t cmd[6];
  rm_mgmt_pack6_stream(addr, value.length(), cmd);
  prov_->output(std::string((char *)&cmd[0], sizeof(cmd)));
  prov_->output(value);
}

/* static */ void Implementation::input_cb_(const std::string &msg,
                                            void *user_data) {
  (void)msg;
  (void)user_data;

  Implementation *that = (Implementation *)user_data;
  that->input_cb_real_(msg);
}

void Implementation::input_cb_real_(const std::string &msg) {
  RCLCPP_DEBUG(node_->get_logger(), "Received data: %s",
               (ros2_serial::utils::bin2hex(msg)).c_str());

  input_queue_mutex_.lock();
  input_queue_ += msg;  // TODO(clairbee): optimize it to reduce extra copying
  RCLCPP_DEBUG(node_->get_logger(), "Queued data: %s",
               (ros2_serial::utils::bin2hex(input_queue_)).c_str());

  while (input_queue_.length() >= LENGTH_CMD_MIN) {
    if ((uint8_t)input_queue_[0] != HEADER_1) {
      input_queue_ = input_queue_.substr(1);
      continue;
    }
    if ((uint8_t)input_queue_[1] != HEADER_2) {
      input_queue_ = input_queue_.substr(2);
      continue;
    }

    if (input_queue_[2] == COMMAND_READ) {
      if (input_queue_.length() < LENGTH_READ_RESP) {
        // Not enough data yet
        break;
      }

      uint16_t addr = ((uint8_t)input_queue_[3]) << 8;
      addr += ((uint8_t)input_queue_[4]);

      if (addr >= ADDR_SERVICE_MIN && addr <= ADDR_SERVICE_MAX) {
        // Handle service traffic within this module.
        // TODO(clairbee): handle ping messages
      } else {
        // Treat everything else as traffic related to one of the accessories.
        auto acc_it = accessories_.find(addr);
        if (acc_it == accessories_.end()) {
          RCLCPP_ERROR(node_->get_logger(),
                       "Received READ from an unknown accessory: %d", addr);
        } else {
          uint16_t value = ((uint8_t)input_queue_[5]) << 8;
          value += ((uint8_t)input_queue_[6]);

          acc_it->second->read_cb(value);
        }
      }

      input_queue_ = input_queue_.substr(LENGTH_READ_RESP);
    } else if (input_queue_[2] == COMMAND_STREAM) {
      if (input_queue_.length() < LENGTH_STREAM_MIN) {
        // Not enough data yet
        break;
      }

      uint8_t len = ((uint8_t)input_queue_[5]);
      if ((int)input_queue_.length() < LENGTH_STREAM_MIN + len) {
        // Not enough data yet
        break;
      }

      uint16_t addr = ((uint8_t)input_queue_[3]) << 8;
      addr += ((uint8_t)input_queue_[4]);

      auto acc_it = accessories_.find(addr);
      if (acc_it == accessories_.end()) {
        RCLCPP_ERROR(node_->get_logger(),
                     "Received STREAM from an unknown accessory: %d", addr);
      } else {
        acc_it->second->stream_cb(
            std::string((char *)&input_queue_[LENGTH_STREAM_MIN], len));
      }

      input_queue_ = input_queue_.substr(LENGTH_STREAM_MIN + len);
    } else {
      RCLCPP_ERROR(node_->get_logger(),
                   "Received unknown command from an unknown accessory: %d",
                   input_queue_[2]);
      input_queue_ = input_queue_.substr(2);  // remove the header only
    }
  }
  input_queue_mutex_.unlock();
}

rclcpp::FutureReturnCode Implementation::reset_handler_(
    const std::shared_ptr<srv::Reset::Request> request,
    std::shared_ptr<srv::Reset::Response> response) {
  response->success = reset_(request->hard, request->reflash);
  return rclcpp::FutureReturnCode::SUCCESS;
}

}  // namespace remote_microcontroller
