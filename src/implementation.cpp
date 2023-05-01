/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_microcontroller/implementation.hpp"

#include <yaml.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <charconv>
#include <locale>

#include "remote_microcontroller/config.hpp"
#include "remote_microcontroller/gpio_switch.hpp"
#include "remote_microcontroller/interface.hpp"
#include "remote_microcontroller/proto_mgmt.hpp"
#include "remote_microcontroller/proto_service.hpp"
#include "remote_microcontroller/puldir_stepper_driver.hpp"
#include "remote_microcontroller/pwm_actuator_position.hpp"
#include "remote_microcontroller/pwm_actuator_velocity.hpp"
#include "remote_microcontroller/uart.hpp"
#include "remote_serial/factory.hpp"
#include "remote_serial/utils.hpp"

namespace remote_microcontroller {

Implementation::Implementation(
    rclcpp::Node *node,
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec)
    : Interface{node}, initialized_{false}, exec_{exec} {
  auto prefix = get_prefix_();
  std::string ns = node_->get_namespace();

  prov_ = remote_serial::Factory::New(node);
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
  auto config_file = fopen(config_filename.c_str(), "r");
  yaml_parser_t parser;
  yaml_document_t doc;
  yaml_parser_initialize(&parser);
  yaml_parser_set_input_file(&parser, config_file);
  if (yaml_parser_load(&parser, &doc)) {
    RCLCPP_DEBUG(node->get_logger(), "YAML file is loaded");
    yaml_node_t *root = yaml_document_get_root_node(&doc);

    if (root && root->type == YAML_MAPPING_NODE) {
      for (auto root_item = root->data.mapping.pairs.start;
           root_item < root->data.mapping.pairs.end; root_item++) {
        RCLCPP_DEBUG(node->get_logger(), "root item is found");
        auto root_key = yaml_document_get_node(&doc, root_item->key);
        if (!root_key || root_key->type != YAML_SCALAR_NODE) continue;

        auto accessories = yaml_document_get_node(&doc, root_item->value);
        if (!accessories || accessories->type != YAML_SEQUENCE_NODE) continue;
        RCLCPP_DEBUG(node->get_logger(), "root item is a sequence");

        std::string chapter((char *)root_key->data.scalar.value);

        int i;
        yaml_node_item_t *accessory_item;
        for (accessory_item = accessories->data.sequence.items.start, i = 0;
             accessory_item < accessories->data.sequence.items.top;
             ++accessory_item, ++i) {
          RCLCPP_DEBUG(node->get_logger(), "accessory item is found");
          auto accessory = yaml_document_get_node(&doc, *accessory_item);
          if (!accessory || accessory->type != YAML_MAPPING_NODE) continue;
          RCLCPP_DEBUG(node->get_logger(), "accessory item is a map");

          std::string node_name;
          std::string node_prefix;
          int channel = i;
          int dir_channel = -1;  // used by puldir only
          std::string type;      // used by pwm only
          auto node_options = rclcpp::NodeOptions{};

          for (auto param_item = accessory->data.mapping.pairs.start;
               param_item < accessory->data.mapping.pairs.end; param_item++) {
            RCLCPP_DEBUG(node->get_logger(), "param item is found: %d",
                         param_item->key);
            auto param_key = yaml_document_get_node(&doc, param_item->key);
            if (!param_key || param_key->type != YAML_SCALAR_NODE) continue;
            RCLCPP_DEBUG(node->get_logger(), "param item is a scalar");
            auto key = std::string((char *)param_key->data.scalar.value);
            RCLCPP_DEBUG(node->get_logger(), "param item is %s",
                         (char *)param_key->data.scalar.value);

            auto param_value = yaml_document_get_node(&doc, param_item->value);
            if (!param_value || param_value->type != YAML_SCALAR_NODE) continue;
            RCLCPP_DEBUG(node->get_logger(), "param value is a scalar");

            if (key == "type") {
              type = std::string((char *)param_value->data.scalar.value);
            } else if (key == "name") {
              node_name = "driver_microcontroller_" +
                          std::string((char *)param_value->data.scalar.value);
            } else if (key == "prefix") {
              node_prefix = std::string((char *)param_value->data.scalar.value);
            } else if (key == "channel") {
              channel = std::atoi((char *)param_value->data.scalar.value);
            } else if (key == "dir_channel") {
              dir_channel = std::atoi((char *)param_value->data.scalar.value);
            } else {
              std::string string_value((char *)param_value->data.scalar.value);
              const char *start = string_value.c_str();
              const char *end = start + strlen(start);

              int int_value;
              auto res = std::from_chars(start, end, int_value);
              if (res.ec == std::errc{} && !*res.ptr) {
                node_options.parameter_overrides().push_back({key, int_value});
              } else {
                double double_value;
#ifdef __clang__
                if (sscanf(start, "%lf", &double_value) == 1)
#else
                res = std::from_chars(start, end, double_value);
                if (res.ec == std::errc{} && !*res.ptr)
#endif
                {
                  node_options.parameter_overrides().push_back(
                      {key, double_value});
                } else {
                  node_options.parameter_overrides().push_back(
                      {key, string_value});
                }
              }
            }
          }

          if (node_name == "") {
            node_name =
                "driver_microcontroller_" + chapter + std::to_string(channel);
          }

          // Create the node
          RCLCPP_DEBUG(node->get_logger(), "creating the node");
          node_options.use_intra_process_comms(true);
          auto accessory_node =
              std::make_shared<rclcpp::Node>(node_name, ns, node_options);
          exec->add_node(accessory_node);

          // Instantiate the accessory
          RCLCPP_DEBUG(node->get_logger(), "instantiating the accessory");
          std::shared_ptr<Accessory> ptr;
          if (chapter == MICROCONTROLLER_CONFIG_CHAPTER_GPIO) {
            ptr = std::make_shared<GPIOSwitch>(accessory_node.get(), this,
                                               channel, node_prefix);
            RCLCPP_DEBUG(node->get_logger(), "instantiated GPIO");
          } else if (chapter == MICROCONTROLLER_CONFIG_CHAPTER_PUL) {
            ptr = std::make_shared<PulDirStepperDriver>(
                accessory_node.get(), this, channel, dir_channel, node_prefix);
            RCLCPP_DEBUG(node->get_logger(),
                         "instantiated PulDirStepperDriver");
          } else if (chapter == MICROCONTROLLER_CONFIG_CHAPTER_PWM) {
            if (type == "actuator_position") {
              RCLCPP_DEBUG(node_->get_logger(),
                           "Found a position actuator entry");
              ptr = std::make_shared<PWMActuatorVelocity>(
                  accessory_node.get(), this, channel, node_prefix);
              RCLCPP_DEBUG(node->get_logger(),
                           "instantiated PWMActuatorVelocity");
            } else if (type == "actuator_position") {
              RCLCPP_DEBUG(node_->get_logger(),
                           "Found a velocity actuator entry");
              ptr = std::make_shared<PWMActuatorPosition>(
                  accessory_node.get(), this, channel, node_prefix);
              RCLCPP_DEBUG(node->get_logger(),
                           "instantiated PWMActuatorPosition");
            } else {
              RCLCPP_DEBUG(node_->get_logger(), "Found a simple pwm entry");
              ptr = std::make_shared<PWM>(accessory_node.get(), this, channel,
                                          node_prefix);
              RCLCPP_DEBUG(node->get_logger(), "instantiated PWM");
            }
          } else if (chapter == MICROCONTROLLER_CONFIG_CHAPTER_UART) {
            ptr = std::make_shared<UART>(accessory_node.get(), this, channel,
                                         node_prefix);
            RCLCPP_DEBUG(node->get_logger(), "instantiated UART");
          }

          // Store the accessory in the collection
          accessories_.insert({ptr->get_addr(), ptr});
        }
      }
    }
    RCLCPP_DEBUG(node->get_logger(), "Destroying the document");
    yaml_document_delete(&doc);
  }
  RCLCPP_DEBUG(node->get_logger(), "Destroying the parser");
  yaml_parser_delete(&parser);
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

void Implementation::read(uint16_t addr) {
  uint8_t cmd[5];
  rm_mgmt_pack5_read(addr, cmd);
  prov_->output(std::string((char *)&cmd[0], sizeof(cmd)));
}

void Implementation::stream(uint16_t addr, const std::string &value) {
  uint8_t cmd[6];
  rm_mgmt_pack6_stream(addr, value.length(), cmd);
  prov_->output(std::string((char *)&cmd[0], sizeof(cmd)) + value);
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
               (remote_serial::utils::bin2hex(msg)).c_str());

  input_queue_mutex_.lock();
  input_queue_ += msg;  // TODO(clairbee): optimize it to reduce extra copying
  RCLCPP_DEBUG(node_->get_logger(), "Queued data: %s",
               (remote_serial::utils::bin2hex(input_queue_)).c_str());

  while (input_queue_.length() >= LENGTH_CMD_MIN) {
    if ((uint8_t)input_queue_[0] != HEADER_1) {
      input_queue_ = input_queue_.substr(1);
      continue;
    }
    if ((uint8_t)input_queue_[1] != HEADER_2) {
      input_queue_ = input_queue_.substr(2);
      continue;
    }

    // With some level of confidence we can assume that we are receiving data
    // from a properly programmed controller.
    // Now it's time to initialize all accessories if it has not been done yet.
    if (!initialized_) {
      for (auto &acc_it : accessories_) {
        acc_it.second->init();
      }
      initialized_ = true;
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
        // Treat everything else as traffic related to one of the
        // accessories.
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
