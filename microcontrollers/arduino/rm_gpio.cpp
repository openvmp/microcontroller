/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "rm_gpio.hpp"

#include "rm_mgmt.hpp"

#define GPIO_CHANNELS_NUM 54

uint8_t gpio_modes[GPIO_CHANNELS_NUM];
#define GPIO_MODE_INPUT 1
#define GPIO_MODE_INPUT_PULLUP 2
#define GPIO_MODE_OUTPUT 3

void rm_gpio_setup() {
  // TODO(clairbee): can we rely on .bss here?
  memset(&gpio_modes, 0, sizeof(gpio_modes));
}

void rm_gpio_read(uint8_t addr) {
  if (addr < ADDR_GPIO_MIN || addr > ADDR_GPIO_MAX) {
    rm_mgmt_report_read(addr, 0xDEAD);
    return;
  }
  uint16_t channel = addr - ADDR_GPIO_MIN;

  // Let's agree to use channel >200 as the pull up version of the first 200
  // channels
  bool pullup = false;
  if (channel >= 200) {
    pullup = true;
    channel -= 100;
  }
  if (channel >= GPIO_CHANNELS_NUM) {
    rm_mgmt_report_read(addr, 0xDEAD);
    return;
  }

  if (!pullup && gpio_modes[channel] != GPIO_MODE_INPUT) {
    pinMode(channel, INPUT);
    gpio_modes[channel] = GPIO_MODE_INPUT;
  } else if (pullup && gpio_modes[channel] != GPIO_MODE_INPUT_PULLUP) {
    pinMode(channel, INPUT_PULLUP);
    gpio_modes[channel] = GPIO_MODE_INPUT_PULLUP;
  }

  bool result = digitalRead(channel);
  rm_mgmt_report_read(addr, result);
}

void rm_gpio_write(uint8_t addr, uint16_t value) {
  if (addr < ADDR_GPIO_MIN || addr > ADDR_GPIO_MAX) {
    // TODO(clairbee): report the error
    return;
  }
  uint16_t channel = addr - ADDR_GPIO_MIN;

  if (channel >= GPIO_CHANNELS_NUM) {
    // TODO(clairbee): report the error
    return;
  }

  if (gpio_modes[channel] != GPIO_MODE_OUTPUT) {
    pinMode(channel, OUTPUT);
    gpio_modes[channel] = GPIO_MODE_OUTPUT;
  }

  digitalWrite(channel, value ? HIGH : LOW);
}
