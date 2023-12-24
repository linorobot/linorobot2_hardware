# linorobot2_hardware with esp32 support

This is a fork of the [linorobot/linorobot2_hardware](https://github.com/linorobot/linorobot2_hardware) project which builds firmware for micro-controllers to control mobile robots based on micro ROS. The esp32 support is added to this fork. 

## Why esp32

The WIFI capability of the esp32 can be used to build low-cost mobile robots with navigation control under ROS2.

In traditional robot builds, there are robot computers (such as raspberry pi, jetson nono or PC) running the build tools and ROS2/NAV2 navigation stack. With the WIFI capability of the esp32, the robot firmware can be developed on your laptop or desktop PC. After the firmware is written to the esp32 using USB cable, the following firmware updates can be performed remotely using ArduinoOTA. The debug messages can be read using remote syslog server. Using the WIFI transport of the micro ROS, the navigation packages NAV2 / SLAM and visualization tool RVIZ2 can be served on your laptop or desktop PC. This is a more comfortable development environment than the more restricted robot computers. No robot computer is required on the robot now. Just an esp32 micro-controller will serve. And the cost of the robot will be a little cheaper.

Please check the [Wiki](https://github.com/hippo5329/linorobot2_hardware/wiki) for configurations, guides and documents.
Questions (via [Discussions](https://github.com/hippo5329/linorobot2_hardware/discussions)),
bug reports (via [Issues](https://github.com/hippo5329/linorobot2_hardware/issues)) and
code contributions (via [Pull requests](https://github.com/hippo5329/linorobot2_hardware/pulls)) are welcome.

## Supported robot types

- 2wd - 2 wheel drive robot.
- 4wd - 4 wheel drive robot.
- mecanum - Mecanum drive robot.

## Supported micro-controllers

- [teensy](https://www.pjrc.com/teensy/) - the original micro-controller supported in the linorobot/linorobot_hardware project. The teensy dose not support WIFI.
- [esp32](https://en.wikipedia.org/wiki/ESP32) - support added in this fork. There are many variants. The RISC-V variant esp32-c3 does not support PCNT (hardware pulse counters), which is needed for encoders. So the esp32-c3 is not recommended.

## Quick start

Install [PlatformIO IDE for VSCode](https://platformio.org/install/ide?install=vscode). The PlatformIO core CLI will be installed automatically.

    sudo apt update
    sudo apt remove brltty -y
    sudo apt install python3-venv build-essential cmake git -y
    sudo apt install software-properties-common apt-transport-https wget -y
    wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add -
    sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
    sudo apt install code -y
    code --install-extension platformio.platformio-ide

 Add platformio to $PATH in ~/.bashrc or ~/.profile .

    PATH="$PATH:$HOME/.platformio/penv/bin"

You may use VSCode to clone the source, select robot configuration (e.g. *esp32tank*), build and upload. Or do it in command lines.

    git clone https://github.com/hippo5329/linorobot2_hardware.git
    cd linorobot2_hardware/firmware
    pio run -e esp32tank -t upload
