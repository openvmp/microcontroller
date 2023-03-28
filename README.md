# OpenVMP

[![License](./apache20.svg)](./LICENSE.txt)

This package is a part of [the OpenVMP project](https://github.com/openvmp/openvmp).
But it's designed to be universal and usable independently from the rest of OpenVMP or in a combination with select OpenVMP packages.

## remote\_microcontroller

This package implements a minimalistic protocol to relay input and output to
and from compatible microcontrollers to leverage their I/O connectivity.

### Features

The high level features:

- I/O connectivity of microcontrollers is shared with other nodes
- All ROS2 messaging overhead resides on the computer running this package
- Microcontroller is exclusively responsible for I/O operations with
  the least overhead possible
- Microcontrollers I/O contacts are mapped to ROS2 services and topics using
  YAML config files

The currently supported I/O:

- [remote_actuator](https://github.com/openvmp/actuator) on PWM pins
  (either position or velocity variants)
- [remote_switch](https://github.com/openvmp/switch) on GPIO pins
- [ros2_serial_bus](https://github.com/openvmp/serial_bus) on UART channels

### Why not micro-ROS?

Unlike this package, micro-ROS implements DDS messaging middleware on the 
microcontroller itself, allowing it to communicate with other ROS2 nodes over 
intermittent and lossy connections.

In some designs where the microcontroller is permanently connected to
a more powerful computer (e.g. Raspberry Pi), there is no benefit in using
the constrained resources of a microcontroller to run DDS and to handle all
the ROS2 messaging overhead.
If the system is designed in a way so that the worst case scenario
traffic does not saturate the permanent channel between the computer and the
microcontroller, then it's better to delegate messaging functionality to the
computer.

If there is no permanent connection to the computer or if the worst case
scenario may saturate the channel (and it is believed that QoS features will
save the day), then it's better to use micro-ROS instead of this package.
