/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "rm_uart.hpp"

#include <SoftwareSerial.h>

#include "config_uart.hpp"
#include "rm_mgmt.hpp"

#define CHANNELS_INVERSE_OFFSET 50

extern Stream *uart[UART_CHANNELS_NUM];
extern uint8_t uart_rx[UART_CHANNELS_NUM];
extern uint8_t uart_tx[UART_CHANNELS_NUM];
uint8_t uart_inverse[UART_CHANNELS_NUM];

#define INPUT_MAX 32 /* max characters to read per loop */
uint8_t *input_buffer;

void rm_uart_setup() {
  // TODO(clairbee): can we rely on .bss here?
  memset(&uart_inverse, 0, sizeof(uart_inverse));
  input_buffer = (uint8_t *)malloc(INPUT_MAX);
}

void rm_uart_stream(uint16_t addr, uint8_t *ptr, uint8_t len) {
  if (addr < ADDR_UART_MIN || addr > ADDR_UART_MAX) {
    // TODO(clairbee): report the error
    return;
  }

  uint8_t channel = addr - ADDR_UART_MIN;
  bool inverse = channel >= CHANNELS_INVERSE_OFFSET;
  channel %= CHANNELS_INVERSE_OFFSET;
  if (channel >= UART_CHANNELS_NUM) {
    // TODO(clairbee): report the error
    return;
  }

  if (uart[channel] == nullptr) {
    uart[channel] =
        new SoftwareSerial(uart_rx[channel], uart_tx[channel], inverse);
    uart_inverse[channel] = inverse;
  } else {
    if (uart_inverse[channel] != inverse) {
      // inverse configuration mismatch
      // TODO(clairbee): report the error
      return;
    }
  }

  uart[channel]->write(ptr, len);
}

void rm_uart_loop() {
  for (int i = 0; i < UART_CHANNELS_NUM; i++) {
    if (uart[i] == nullptr) {
      continue;
    }

    if (!uart[i]->available()) {
      continue;
    }

    int8_t input_offset = 0;
    while (uart[i]->available() && input_offset < INPUT_MAX) {
      input_buffer[input_offset++] = uart[i]->read();
    }

    uint16_t addr = ADDR_UART_MIN + i;
    if (uart_inverse[i]) {
      addr += CHANNELS_INVERSE_OFFSET;
    }
    rm_mgmt_report_stream(addr, input_buffer, input_offset);
  }
}