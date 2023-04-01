/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-28
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_REMOTE_MICROCONTROLLER_ARDUINO_SERVICE_HPP_INCLUDED
#define OPENVMP_REMOTE_MICROCONTROLLER_ARDUINO_SERVICE_HPP_INCLUDED

#include <Arduino.h>

#include "../../include/remote_microcontroller/proto_uart.hpp"

extern "C" {
extern void rm_service_setup();
extern void rm_service_loop();
}

#endif  // OPENVMP_REMOTE_MICROCONTROLLER_ARDUINO_SERVICE_HPP_INCLUDED

