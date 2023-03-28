/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "config_pwm.hpp"

#include <Arduino.h>

static uint8_t pwm_channel_to_pin_map[6] = {3, 5, 6, 9, 10, 11};

uint8_t pwm_channel_to_pin(uint8_t channel) {
  return pwm_channel_to_pin_map[channel];
}