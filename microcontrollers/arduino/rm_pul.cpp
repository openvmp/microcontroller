/*
 * OpenVMP, 2023
 *
 * Author: Roman Kuzmenko
 * Created: 2023-04-09
 *
 * Licensed under Apache License, Version 2.0.
 */

#include "rm_pul.hpp"

#include <Arduino.h>

#include "config_pul.hpp"
#include "main.hpp"

struct Pul {
  unsigned long micros_between_states;  // half of time between pulses
  unsigned long micros_state_next;      // next time the state must change
#define PUL_STATE_UNINITIALIZED 0
#define PUL_STATE_RUNNING 1
#define PUL_STATE_STOPPED 2
  uint8_t state;
  uint8_t on;       // last output state
  uint8_t stopped;  // last output state
  int16_t next;     // the index of the next one or -1
};

Pul pul[PUL_CHANNELS_NUM];

// Linked list
int head = -1;
int *tail = nullptr;  // The pointer to the "next" field of the tail of queue

extern uint8_t pul_channel_to_pin(uint8_t channel);

void rm_pul_setup() {
  // TODO(clairbee): can we rely on .bss here?
  memset(&pul, 0, sizeof(pul));
}

void rm_pul(uint8_t addr, uint16_t value) {
  uint8_t channel = addr - ADDR_PUL_MIN;
  if (channel < 0 || channel >= PUL_CHANNELS_NUM) {
    return;
  }

  uint16_t pin = pul_channel_to_pin(channel);

  if (value == 0) {
    pul[channel].state = PUL_STATE_STOPPED;
    return;
  }
  pul[channel].stopped = 0;

  unsigned long delta = 500000 / value;

  if (pul[channel].state == PUL_STATE_UNINITIALIZED) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);

    pul[channel].micros_between_states = delta;
    pul[channel].micros_state_next = now + pul[channel].micros_between_states;
    pul[channel].on = 1;
    pul[channel].next = -1;

    if (tail == nullptr) {
      head = channel;
    } else {
      *tail = channel;
    }
    tail = &pul[channel].next;
  } else {
    // when did we change the state last time
    pul[channel].micros_state_next -= pul[channel].micros_between_states;
    // how often do we need to change states now
    pul[channel].micros_between_states = delta;
    // when will we change the state next time
    pul[channel].micros_state_next += pul[channel].micros_between_states;

    if (pul[channel].micros_state_next < now) {
      pul[channel].micros_state_next = now;
    }
  }

  pul[channel].state = PUL_STATE_RUNNING;
}

void rm_pul_loop() {
  int next = head;
  while (next != -1) {
    if (pul[next].state == PUL_STATE_RUNNING &&
        now >= pul[next].micros_state_next) {
      pul[next].micros_state_next += pul[next].micros_between_states;
      pul[next].on = !pul[next].on;
      digitalWrite(next, pul[next].on ? HIGH : LOW);
    }
    next = pul[next].next;
  }
}