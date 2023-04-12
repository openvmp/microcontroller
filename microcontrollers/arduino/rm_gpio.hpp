/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_REMOTE_MICROCONTROLLER_ARDUINO_GPIO_HPP_INCLUDED
#define OPENVMP_REMOTE_MICROCONTROLLER_ARDUINO_GPIO_HPP_INCLUDED

#include <Arduino.h>

#include "../../include/remote_microcontroller/proto_gpio.hpp"

extern void rm_gpio_setup();
extern void rm_gpio_read(uint8_t addr);
extern void rm_gpio_write(uint8_t addr, uint16_t value);

#endif  // OPENVMP_REMOTE_MICROCONTROLLER_ARDUINO_GPIO_HPP_INCLUDED