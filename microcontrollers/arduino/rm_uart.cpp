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

extern HardwareSerial *uart_hw[UART_HW_CHANNELS_NUM];
extern SoftwareSerial *uart_sw[UART_SW_CHANNELS_NUM];
extern Stream **uart_ptr[UART_CHANNELS_NUM];
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

void rm_uart_write(uint8_t addr, uint16_t value) {
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

  if ((*uart_ptr[channel]) == nullptr) {
    (*uart_ptr[channel]) =
        new SoftwareSerial(uart_rx[channel], uart_tx[channel], inverse);
    uart_inverse[channel] = inverse;
  } else {
    if (uart_inverse[channel] != inverse) {
      // inverse configuration mismatch
      // TODO(clairbee): report the error
      return;
    }
  }

  unsigned long baud_rate = 0;
  switch (value) {
    case 1:
      baud_rate = 300;
      break;
    case 2:
      baud_rate = 600;
      break;
    case 3:
      baud_rate = 1200;
      break;
    case 4:
      baud_rate = 2400;
      break;
    case 5:
      baud_rate = 4800;
      break;
    case 6:
      baud_rate = 9600;
      break;
    case 7:
      baud_rate = 14400;
      break;
    case 8:
      baud_rate = 19200;
      break;
    case 9:
      baud_rate = 28800;
      break;
    case 10:
      baud_rate = 31250;
      break;
    case 11:
      baud_rate = 38400;
      break;
    case 12:
      baud_rate = 57600;
      break;
    case 13:
      baud_rate = 76800;
      break;
    case 14:
      baud_rate = 115200;
      break;
  }

  if (baud_rate == 0) {
    return;
  }

  if (channel < UART_HW_CHANNELS_NUM) {
    uart_hw[channel]->begin(baud_rate);
  } else {
    uart_sw[channel - UART_HW_CHANNELS_NUM]->begin(baud_rate);
  }
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

  if ((*uart_ptr[channel]) == nullptr) {
    (*uart_ptr[channel]) =
        new SoftwareSerial(uart_rx[channel], uart_tx[channel], inverse);
    uart_inverse[channel] = inverse;
  } else {
    if (uart_inverse[channel] != inverse) {
      // inverse configuration mismatch
      // TODO(clairbee): report the error
      return;
    }
  }

  (*uart_ptr[channel])->write(ptr, len);
}

void rm_uart_loop() {
  for (int i = 0; i < UART_CHANNELS_NUM; i++) {
    if ((*uart_ptr[i]) == nullptr) {
      continue;
    }

    if (!(*uart_ptr[i])->available()) {
      continue;
    }

    int8_t input_offset = 0;
    do {
      input_buffer[input_offset++] = (*uart_ptr[i])->read();
    } while ((*uart_ptr[i])->available() && input_offset < INPUT_MAX);

    uint16_t addr = ADDR_UART_MIN + i;
    if (uart_inverse[i]) {
      addr += CHANNELS_INVERSE_OFFSET;
    }
    rm_mgmt_report_stream(addr, input_buffer, input_offset);
  }
}
