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
#include <SoftwareSerial.h>

HardwareSerial *uart_hw[UART_HW_CHANNELS_NUM] = {
    &Serial1,
    &Serial2,
    &Serial3,
};
SoftwareSerial *uart_sw[UART_SW_CHANNELS_NUM] = {
    nullptr,
    nullptr,
};
Stream **uart_ptr[UART_CHANNELS_NUM] = {
    &uart_hw[0], &uart_hw[1], &uart_hw[2], &uart_sw[0], &uart_sw[1],
};
uint8_t uart_rx[UART_CHANNELS_NUM] = {19, 17, 15, 50, 52};
uint8_t uart_tx[UART_CHANNELS_NUM] = {18, 16, 14, 51, 53};
