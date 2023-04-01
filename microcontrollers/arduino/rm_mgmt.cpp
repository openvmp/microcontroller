/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "rm_mgmt.hpp"

#include "rm_gpio.hpp"
#include "rm_pwm.hpp"
#include "rm_uart.hpp"

#define MGMT_INPUT_HEADER_1 0
#define MGMT_INPUT_HEADER_2 1
#define MGMT_INPUT_COMMAND 2
#define MGMT_INPUT_ADDR_1 3
#define MGMT_INPUT_ADDR_2 4
#define MGMT_INPUT_VALUE_1 5
#define MGMT_INPUT_VALUE_2 6
#define MGMT_INPUT_LENGTH 7
#define MGMT_INPUT_STRING 8
int mgmt_input_state = MGMT_INPUT_HEADER_1;

uint8_t command;

uint16_t addr;
uint16_t value;

uint8_t length;
uint8_t bytes_left;
uint8_t *stream_data;
uint16_t stream_data_size;

extern "C" {

void rm_mgmt_setup() {
  stream_data_size = 256;
  stream_data = (uint8_t *)malloc(stream_data_size);

  Serial.begin(115200);
}

void rm_mgmt_loop() {
  uint8_t byte = Serial.read();

  switch (mgmt_input_state) {
    case MGMT_INPUT_HEADER_1:
      if (byte == HEADER_1) {
        mgmt_input_state = MGMT_INPUT_HEADER_2;
      }
      break;

    case MGMT_INPUT_HEADER_2:
      if (byte == HEADER_2) {
        mgmt_input_state = MGMT_INPUT_COMMAND;
      } else {
        // FIXME(clairbee): report the error
        mgmt_input_state = MGMT_INPUT_HEADER_1;
      }
      break;

    case MGMT_INPUT_COMMAND:
      command = byte;
      switch (command) {
        case COMMAND_WRITE:
        case COMMAND_READ:
        case COMMAND_STREAM:
          mgmt_input_state = MGMT_INPUT_ADDR_1;
          break;
        default:
          mgmt_input_state = MGMT_INPUT_HEADER_1;
      }
      break;

    case MGMT_INPUT_ADDR_1:
      addr = byte << 8;
      mgmt_input_state = MGMT_INPUT_ADDR_2;
      break;

    case MGMT_INPUT_ADDR_2:
      addr += byte;
      switch (command) {
        case COMMAND_WRITE:
          mgmt_input_state = MGMT_INPUT_VALUE_1;
          break;
        case COMMAND_READ:
          if (addr >= ADDR_GPIO_MIN && addr <= ADDR_GPIO_MAX) {
            rm_gpio_read(addr);
          } else {
            rm_mgmt_report_read(addr, 0xDEAD);
          }
          mgmt_input_state = MGMT_INPUT_HEADER_1;
          break;
        case COMMAND_STREAM:
          mgmt_input_state = MGMT_INPUT_LENGTH;
          break;
      }
      break;

    case MGMT_INPUT_VALUE_1:
      value = byte << 8;
      mgmt_input_state = MGMT_INPUT_VALUE_2;
      break;

    case MGMT_INPUT_VALUE_2:
      value += byte;

      if (addr >= ADDR_PWM_MIN && byte <= ADDR_PWM_MAX) {
        rm_pwm(addr, value);
      } else if (addr >= ADDR_GPIO_MIN && byte <= ADDR_GPIO_MAX) {
        rm_gpio_write(addr, value);
      } else {
        // TODO(clairbee): report the error
      }
      mgmt_input_state = MGMT_INPUT_HEADER_1;
      break;

    case MGMT_INPUT_LENGTH:
      length = byte;
      bytes_left = length;
      if (length == 0) {
        // TODO(clairbee): subscribe for inbound data
        mgmt_input_state = MGMT_INPUT_HEADER_1;
      } else {
        if (length > stream_data_size) {
          while (length > stream_data_size) {
            stream_data_size *= 1.5;
          }
          stream_data = (uint8_t *)realloc(stream_data, stream_data_size);
        }
        mgmt_input_state = MGMT_INPUT_STRING;
      }
      break;

    case MGMT_INPUT_STRING:
      stream_data[length - bytes_left] = byte;
      bytes_left--;

      if (bytes_left == 0) {
        rm_uart_stream(addr, &stream_data[0], length);
        mgmt_input_state = MGMT_INPUT_HEADER_1;
      }
      break;

    default:
      // FIXME(clairbee): report the error
      mgmt_input_state = MGMT_INPUT_HEADER_1;
  }
}

void rm_mgmt_report_read(uint16_t addr, uint16_t value) {
  uint8_t response[7];
  rm_mgmt_pack7_value(COMMAND_READ, addr, value, response);
  Serial.write(&response[0], sizeof(response));
}

void rm_mgmt_report_stream(uint16_t addr, uint8_t *ptr, uint8_t len) {
  uint8_t response[6];
  rm_mgmt_pack6_stream(addr, len, response);
  Serial.write(&response[0], sizeof(response));
  Serial.write(ptr, len);
}

}  // extern "C"
