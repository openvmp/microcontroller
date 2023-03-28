/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_REMOTE_MICROCONTROLLER_ARDUINO_MGMT_HPP_INCLUDED
#define OPENVMP_REMOTE_MICROCONTROLLER_ARDUINO_MGMT_HPP_INCLUDED

#include <Arduino.h>

#include "../../include/remote_microcontroller/proto_mgmt.hpp"

extern "C" {
void rm_mgmt_setup();
void rm_mgmt_loop();
void rm_mgmt_report_read(uint16_t addr, uint16_t value);
void rm_mgmt_report_stream(uint16_t addr, uint8_t *ptr, uint8_t len);
}

#endif  // OPENVMP_REMOTE_MICROCONTROLLER_ARDUINO_MGMT_HPP_INCLUDED