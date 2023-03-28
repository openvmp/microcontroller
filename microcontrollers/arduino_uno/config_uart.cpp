/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "config_uart.hpp"

#include <Arduino.h>

Stream *uart[UART_CHANNELS_NUM] = {nullptr};
uint8_t uart_rx[UART_CHANNELS_NUM] = {2};
uint8_t uart_tx[UART_CHANNELS_NUM] = {3};
