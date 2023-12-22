# linorobot2_hardware

This is a fork of the [linorobot/linorobot2_hardware](https://github.com/linorobot/linorobot2_hardware) project which builds firmware for micro-controllers to cotrol mobile robots based on micro ROS. The esp32 support is added to this fork. The WIFI capability of the esp32 can be used to build low-cost mobile robots with navigation control under ROS2.

Please check the [Wiki](https://github.com/hippo5329/linorobot2_hardware/wiki) for guides and documents.
Questions (via [Discussions](https://github.com/hippo5329/linorobot2_hardware/discussions)),
bug reports (via [Issues](https://github.com/hippo5329/linorobot2_hardware/issues)) and
code contributions (via [Pull requests](https://github.com/hippo5329/linorobot2_hardware/pulls)) are welcome.

Video clip of [A ROS2 robot tank built on esp32. Navigating..](https://www.youtube.com/watch?v=wmI4SzyXW6o)

## Supported robot types

- 2wd - 2 wheel drive robot.
- 4wd - 4 wheel drive robot.
- mecanum - Mecanum drive robot.

## Supported micro-controllers

- [teensy](https://www.pjrc.com/teensy/): the original micro-contoller supported in the linorobot/linorobot_hardware project. The teensy dose not support WIFI.
- [esp32](https://en.wikipedia.org/wiki/ESP32): support added in this fork. There are many varients. The RISC-V varient esp32-c3 does not support PCNT (hardware pulse counters), which is needed for encoders. So the esp32-c3 is not recommanded.

Please note, the pico-w is not supported yet. I added experimental configuration and code for pico-w. But they are not tested. I am not familiar with pico-w and I don't have pico-w boards to test. The size of memory storage on the pico-w might be too low to run micro ROS WIFI transport.

## Quick start

Install [PlatformIO IDE for VSCode](https://platformio.org/install/ide?install=vscode). The PlatformIO core CLI will be installed automatically.

Clone the source.

    git clone https://github.com/hippo5329/linorobot2_hardware.git

Select robot configuration (eg. *esp32tank*), build and upload.

    cd linorobot2_hardware/firmware
    pio run -e esp32tank -t upload
