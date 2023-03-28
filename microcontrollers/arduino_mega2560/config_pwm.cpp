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

uint8_t pwm_channel_to_pin(uint8_t channel) {
  if (channel >= 12) {
    return 44 + channel - 12;
  } else {
    return 2 + channel;
  }
}