/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-26
 *
 * Licensed under Apache License, Version 2.0.
 */

#ifndef OPENVMP_REMOTE_MICROCONTROLLER_MGMT_HPP_INCLUDED
#define OPENVMP_REMOTE_MICROCONTROLLER_MGMT_HPP_INCLUDED

#define HEADER_1 0x97
#define HEADER_2 0x39

#define COMMAND_WRITE 0   // write 2 bytes
#define COMMAND_READ 1    // read 2 bytes
#define COMMAND_STREAM 2  // write <length> bytes

#define LENGTH_READ_REQ 5
#define LENGTH_READ_RESP 7
#define LENGTH_WRITE_REQ 7
#define LENGTH_STREAM_MIN 6

static inline void rm_mgmt_pack5_read(uint16_t addr, uint8_t response[5]) {
  response[0] = HEADER_1;
  response[1] = HEADER_2;
  response[2] = COMMAND_READ;
  response[3] = addr >> 8;
  response[4] = addr;
}

static inline void rm_mgmt_pack7_write(uint16_t addr, uint16_t value,
                                       uint8_t response[7]) {
  response[0] = HEADER_1;
  response[1] = HEADER_2;
  response[2] = COMMAND_WRITE;
  response[3] = addr >> 8;
  response[4] = addr;
  response[5] = value >> 8;
  response[6] = value;
}

static inline void rm_mgmt_pack7_value(uint8_t command, uint16_t addr,
                                       uint16_t value, uint8_t *response) {
  response[0] = HEADER_1;
  response[1] = HEADER_2;
  response[2] = command;
  response[3] = addr >> 8;
  response[4] = addr;
  response[5] = value >> 8;
  response[6] = value;
}

static inline void rm_mgmt_pack6_stream(uint16_t addr, uint8_t len,
                                        uint8_t *response) {
  response[0] = HEADER_1;
  response[1] = HEADER_2;
  response[2] = COMMAND_STREAM;
  response[3] = addr >> 8;
  response[4] = addr;
  response[5] = len;
}

#endif  // OPENVMP_REMOTE_MICROCONTROLLER_MGMT_HPP_INCLUDED