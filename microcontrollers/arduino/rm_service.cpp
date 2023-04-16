/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-03-28
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "rm_service.hpp"

#include "../../include/remote_microcontroller/proto_service.hpp"
#include "rm_mgmt.hpp"

uint32_t service_cycle;

void rm_service_setup() {
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
}

void rm_service_loop() {
  service_cycle++;
  if (service_cycle % 100000 == 0) {
    digitalWrite(13, HIGH - digitalRead(13));  // toggle led
    rm_mgmt_report_read(ADDR_SERVICE_PULSE, service_cycle / 100000);
  }
}
