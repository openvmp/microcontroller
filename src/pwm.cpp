/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-27
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "remote_microcontroller/pwm.hpp"

namespace remote_microcontroller {

PWM::PWM(remote_microcontroller::Implementation *microcontroller, int index,
         const std::string &prefix)
    : Accessory(microcontroller, ((uint16_t)(ADDR_PWM_MIN + index)), prefix) {}

void PWM::read_cb(uint16_t value) {
  (void)value;
  // PWM does not use register read/write ops
}
void PWM::stream_cb(const std::string &value) {
  (void)value;
  // PWM does not use stream ops
}

}  // namespace remote_microcontroller
