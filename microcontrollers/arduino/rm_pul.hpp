/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-04-09
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_REMOTE_MICROCONTROLLER_ARDUINO_PUL_HPP_INCLUDED
#define OPENVMP_REMOTE_MICROCONTROLLER_ARDUINO_PUL_HPP_INCLUDED

#include <Arduino.h>

#include "../../include/remote_microcontroller/proto_pul.hpp"

extern void rm_pul_setup();
extern void rm_pul(uint8_t addr, uint16_t value);
extern void rm_pul_loop();

#endif  // OPENVMP_REMOTE_MICROCONTROLLER_ARDUINO_PUL_HPP_INCLUDED