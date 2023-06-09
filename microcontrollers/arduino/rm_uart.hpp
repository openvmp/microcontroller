/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_REMOTE_MICROCONTROLLER_ARDUINO_UART_HPP_INCLUDED
#define OPENVMP_REMOTE_MICROCONTROLLER_ARDUINO_UART_HPP_INCLUDED

#include <Arduino.h>

#include "../../include/remote_microcontroller/proto_uart.hpp"

extern void rm_uart_setup();
extern void rm_uart_write(uint8_t addr, uint16_t value);
extern void rm_uart_stream(uint16_t addr, uint8_t *ptr, uint8_t len);
extern void rm_uart_loop();

#endif  // OPENVMP_REMOTE_MICROCONTROLLER_ARDUINO_UART_HPP_INCLUDED
