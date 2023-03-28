/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_REMOTE_MICROCONTROLLER_ARDUINO_PWM_HPP_INCLUDED
#define OPENVMP_REMOTE_MICROCONTROLLER_ARDUINO_PWM_HPP_INCLUDED

#include <Arduino.h>

#include "../../include/remote_microcontroller/proto_pwm.hpp"

extern "C" {
extern void rm_pwm_setup();
extern void rm_pwm(uint8_t addr, uint16_t value);
}

#endif  // OPENVMP_REMOTE_MICROCONTROLLER_ARDUINO_PWM_HPP_INCLUDED