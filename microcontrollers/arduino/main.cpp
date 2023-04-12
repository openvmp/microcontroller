/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "main.hpp"

#include <Arduino.h>

#include "rm_gpio.hpp"
#include "rm_mgmt.hpp"
#include "rm_pul.hpp"
#include "rm_pwm.hpp"
#include "rm_service.hpp"
#include "rm_uart.hpp"

unsigned long now;

void setup() {
  pinMode(13, OUTPUT);

  rm_gpio_setup();
  rm_pul_setup();
  rm_pwm_setup();
  rm_service_setup();
  rm_uart_setup();

  rm_mgmt_setup();
}

void loop() {
  now = micros();

  while (Serial.available() > 0) {
    rm_mgmt_loop();
    digitalWrite(13, HIGH - digitalRead(13));  // toggle led
  }
  rm_pul_loop();
  rm_uart_loop();
  rm_service_loop();

  // TODO(clairbee): consider making delays board-specific
  // delay(1);
}
