# remote_microcontroller

This document enumerates the supported microcontrollers, how to connect it to the computer running this package and what I/O pins are used for what purpose.

## Arduino

All arduino boards have to be connected to the computer using the USB port.
Don't forget to supply additional power (whether through the power port or
the Vin port).

Please, note, all pins are listed in the order of channels (starting with the channel 0).

### Arduino Mega2560

There are 15 PWM channels:

- 2-13
- 44-46

There are 5 UART channels:

- RX: D19, TX: D18
- RX: D17, TX: D16
- RX: D15, TX: D14
- RX: D50, TX: D51
- RX: D52, TX: D53

### Arduino Uno

There are 6 PWM channels:

- 3, 5, 6, 9, 10, 11

There is 1 UART channel:

- RX: D2, TX: D3