/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#include <Arduino.h>

#include "rm_gpio.hpp"
#include "rm_mgmt.hpp"
#include "rm_pwm.hpp"
#include "rm_uart.hpp"

extern "C" {

void setup() {
  pinMode(13, OUTPUT);

  rm_mgmt_setup();
  rm_pwm_setup();
  rm_gpio_setup();
  rm_uart_setup();
}

void loop() {
  while (Serial.available() > 0) {
    rm_mgmt_loop();
    digitalWrite(13, HIGH - digitalRead(13));  // toggle led
  }
  rm_uart_loop();

  delay(1);
}

}  // extern "C"