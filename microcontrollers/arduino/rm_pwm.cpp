/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "rm_pwm.hpp"

#include <Arduino.h>
#include <Servo.h>

#include "config_pwm.hpp"

Servo pwm[PWM_CHANNELS_NUM];
uint8_t pwm_initialized[PWM_CHANNELS_NUM];

extern uint8_t pwm_channel_to_pin(uint8_t channel);

extern "C" {

extern void rm_pwm_setup() {
  // TODO(clairbee): can we rely on .bss here?
  memset(&pwm_initialized, 0, sizeof(pwm_initialized));
}

extern void rm_pwm(uint8_t addr, uint16_t value) {
  uint8_t channel = addr - ADDR_PWM_MIN;

  if (channel < 0 || channel >= PWM_CHANNELS_NUM) {
    return;
  }

  if (pwm_initialized[channel] == 0) {
    pwm[channel].attach(pwm_channel_to_pin(channel));
    pwm_initialized[channel] = 1;
  }

  pwm[channel].write(value);  // should be from 0 to 180
}

}  // extern "C"