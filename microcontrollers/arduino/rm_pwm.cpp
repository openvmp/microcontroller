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

Servo servo[PWM_CHANNELS_NUM];
uint8_t pwm_initialized[PWM_CHANNELS_NUM];
uint8_t servo_library_initialized;
uint8_t pins_9_10_initialized;

extern uint8_t pwm_channel_to_pin(uint8_t channel);

extern "C" {

extern void rm_pwm_setup() {
  // We can only use the Servo library if the PWM pins 9 and 10 are not used
  servo_library_initialized = 0;
  pins_9_10_initialized = 0;

  // TODO(clairbee): can we rely on .bss here?
  memset(&pwm_initialized, 0, sizeof(pwm_initialized));
}

extern void rm_pwm(uint8_t addr, uint16_t value) {
  uint8_t channel = addr - ADDR_PWM_MIN;

  if (channel < 0 || channel >= PWM_CHANNELS_NUM) {
    return;
  }

  if (channel < PWM_CHANNELS_PWM_NUM) {
    // Hardware control of a PWM pin
    uint16_t pin = pwm_channel_to_pin(channel);

    if (pwm_initialized[channel] == 0) {
      if (pin == 9 || pin == 10) {
        if (servo_library_initialized != 0) {
          // TODO(clairbee): report an error
          return;
        }
        pins_9_10_initialized = 1;
      }

      pinMode(pin, OUTPUT);

      pwm_initialized[channel] = 1;
    }

    analogWrite(pin, value);  // should be from 0 to 255
  } else {
    // Software control of an arbitrary digital pin using the Servo library
    uint16_t pin = channel - PWM_CHANNELS_PWM_NUM;

    if (pwm_initialized[channel] == 0) {
      if (pins_9_10_initialized != 0) {
        // TODO(clairbee): report an error
        return;
      }

      servo[channel].attach(pin);
      servo_library_initialized = 1;

      pwm_initialized[channel] = 1;
    }

    servo[channel].write(value);  // should be from 0 to 180
  }
}

}  // extern "C"