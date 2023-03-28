/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-27
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_REMOTE_MICROCONTROLLER_ACCESSORY_H
#define OPENVMP_REMOTE_MICROCONTROLLER_ACCESSORY_H

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "remote_microcontroller/implementation.hpp"

namespace remote_microcontroller {

class Accessory {
 public:
  Accessory(Implementation* microcontroller, uint16_t addr,
            const std::string& prefix);
  virtual ~Accessory(){};

  uint16_t get_addr() const { return addr_; }

  virtual void read_cb(uint16_t value) = 0;
  virtual void stream_cb(const std::string& value) = 0;

 protected:
  void write(uint16_t value);
  void stream(const std::string& value);

 private:
  Implementation* microcontroller_;
  uint16_t addr_;
  const std::string prefix_;
};

}  // namespace remote_microcontroller

#endif  // OPENVMP_REMOTE_MICROCONTROLLER_ACCESSORY_H
