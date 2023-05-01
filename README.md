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

The currently supported I/O channels:

- PWM
  - Arbitrary position and velocity control using the ROS2 built-in std\_msgs::msg::UInt16
  - [remote_actuator](https://github.com/openvmp/actuator)
    - both position and velocity variants are supported
    - see [remote_hardware_interface](https://github.com/openvmp/remote_hardware_interface) for info on how to consume this interface
- PUL and PUL/DIR pulse train generation
  - Arbitrary velocity control using the ROS2 built-in std\_msgs::msg::Int32 (pulses per second, sign is the direction)
  - [remote_actuator](https://github.com/openvmp/actuator) for velocity control
    - see [remote_hardware_interface](https://github.com/openvmp/remote_hardware_interface) for info on how to consume this interface
  - [remote_stepper_driver](https://github.com/openvmp/stepper_driver)
    - pulses per revolution and other parameter tuning
    - provides the [remote_actuator](https://github.com/openvmp/actuator) interface too
- GPIO
  - Arbitrary state control using the ROS2 built-in std\_msgs::msg::Boolean
  - [remote_switch](https://github.com/openvmp/switch)
- UART ports
  - Arbitrary serial input and output using the ROS2 built-in std\_msgs::msg::String
  - [remote_serial](https://github.com/openvmp/serial)

### Supported microcontrollers

It currently supports Arduino Mega2560 and Arduino Uno.
But it was only tested on Arduino Mega2560.

Support for other Arduino boards can be added with trivial changes.
Support for other microcontroller boards would require a little more effort.
Contributors are very welcome!

See the details of mapping I/O channels to the pins of the microcontroller
[here](./microcontrollers/README.md).

### Getting started

Since this package and some of its dependencies are not yet added to ROS2
distribution packages, you will have to clone all of them into the `src`
folder of your ROS2 workspace.

```bash
mkdir src
git clone https://github.com/openvmp/actuator.git src/remote_actuator
git clone https://github.com/openvmp/encoder.git src/remote_encoder
git clone https://github.com/openvmp/switch.git src/remote_switch
git clone https://github.com/openvmp/serial.git src/remote_serial
git clone https://github.com/openvmp/microcontroller.git src/remote_microcontroller
```

After building the workspace, you can link against this package as a library

```c++
// In the declaration of your node class
std::shared_ptr<remote_microcontroller::Implementation> mcu_;

/// In the definition of your node class
#include <remote_microcontroller/factory.hpp>
...
mcu_ = remote_microcontroller::Factory::New(this, exec); // where exec is the multithreaded executor your node is running in
```

or run it as a standalone process:

```bash
ros2 run remote_microcontroller remote_microcontroller_standalone ...
```

In both cases the following parameters need to be provided:

- microcontroller_config: path to the configuration file
- serial_is_remote: false
- serial_dev_name: i.e. /dev/ttyACM0
- serial_baud_rate: i.e. 115200

See [remote_serial](https://github.com/openvmp/serial/blob/main/README.md) for more serial port parameters if needed.

Here is an example of the configuration file for controlling a single servo:

```yaml
pwm:
  - channel: 0 # maps to pin 2 on Arduino Mega2560 and pin 3 on Arduino Uno
    type: simple_pwm # fixed value, one of several supported types
    name: actuator0 # this will be used to produce the node name, no need to match with any other values
    prefix: /pwm0 # where to expose ROS2 interfaces
    pwm_min: 0
    pwm_max: 255
```

If this configuration file is used, then the following command can be used to control the servo motor:

```bash
ros2 topic pub /pwm0/pwm std_msgs/msg/UInt16 '{"data":150}'
```

### Integration with `ros2_control`

The following packages are needed to integrate with `ros2_control`:

```bash
git clone https://github.com/openvmp/remote_hardware_interface.git src/remote_hardware_interface
```

Add the following section to the URDF file:

```xml
  <ros2_control name="HardwareSystem" type="system">
    <hardware>
      <plugin>remote_hardware_interface/RemoteSystemInterface</plugin>
      <param name="namespace">/robot</param>
    </hardware>
    <joint name="joint0">
      <command_interface name="velocity" />
    </joint>
  </ros2_control>
```

Add the following into `ros2_controllers.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 20 # match this value with your performance expectations

    velocity_controller:
      type: velocity_controllers/JointGroupVelocityController

velocity_controller:
  ros__parameters:
    joints:
      - joint0
```

Use the following configuration file (note that `simple_pwm` is replaced with `actuator_position` which provides the interface consumed by `remote_hardware_interface`)

```yaml
pwm:
  - channel: 0 # maps to the pin 2 on Mega2560 and the pin 3 on Uno
    type: actuator_velocity # can be "actuator_position" depending on the type of servo
    name: joint0 # this will be used to produce the node name, no need to match with any other values
    prefix: /robot/actuator/joint0 # namespace + "/actuator/" + joint name
    actuator_velocity_min: -3.14 # or "actuator_position_min"
    actuator_velocity_max: 3.14 # or "actuator_position_max"
    pwm_min: 0
    pwm_max: 255
```

Please, note, if the channel is not specified explicitly
then the natural order of channels is assumed
(the first YAML entry is the channel 0,
the second one is the channel 1 etc).

Now launch your robot and let `ros2_control` do the work.

```bash
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray '{"data":[1.0]}'
```

### Why not micro-ROS?

Unlike this package, micro-ROS implements DDS messaging middleware on the 
microcontroller itself, allowing it to communicate with other ROS2 nodes over 
intermittent and lossy connections.

In some designs where the microcontroller is permanently connected to
a more powerful computer (even a Raspberry Pi), there is no benefit in using
the constrained resources of a microcontroller to run DDS and to handle all
the ROS2 messaging overhead.
If the system is designed in a way so that the worst case scenario
traffic does not saturate the permanent channel between the computer and the
microcontroller, then it's better to delegate messaging functionality to the
computer.

If there is no permanent connection to the computer or if the worst case
scenario may saturate the channel (and it is believed that QoS features will
save the day), then it's better to use micro-ROS instead of this package.
