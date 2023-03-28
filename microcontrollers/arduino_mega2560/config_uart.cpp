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

Stream *uart[UART_CHANNELS_NUM] = {
    &Serial1, &Serial2, &Serial3, nullptr, nullptr,
};
uint8_t uart_rx[UART_CHANNELS_NUM] = {19, 17, 15, 50, 52};
uint8_t uart_tx[UART_CHANNELS_NUM] = {18, 16, 14, 51, 53};
