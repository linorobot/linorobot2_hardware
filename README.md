# linorobot2_hardware with esp32 support

This is a fork of the [linorobot/linorobot2_hardware](https://github.com/linorobot/linorobot2_hardware) project which builds firmware for micro-controllers to control mobile robots based on micro ROS. The esp32 support is added to this fork.

## Why esp32

The WIFI capability of the esp32 can be used to build low-cost mobile robots with navigation control under ROS2.

In traditional robot builds, there are robot computers (such as raspberry pi, jetson nono or mini PC) running the build tools and ROS2/NAV2 navigation stack. With the WIFI capability of the esp32, the robot firmware can be developed on your laptop or desktop PC. After the firmware is written to the esp32 using USB cable, the following firmware updates can be performed remotely using ArduinoOTA. The debug messages can be read using remote syslog server. Using the WIFI transport of the micro ROS, the navigation packages NAV2 / SLAM and visualization tool RVIZ2 can be served on your laptop or desktop PC. This is a more comfortable development environment than the more restricted robot computers. No robot computer is required on the robot now. Just an esp32 micro-controller will serve. And the cost of the robot will be a little cheaper.

## Supported robot types

- 2wd - 2 wheel drive robot.
- 4wd - 4 wheel drive robot.
- mecanum - Mecanum drive robot.

## Supported micro-controllers

- [teensy](https://www.pjrc.com/teensy/) - the original micro-controller supported in the linorobot/linorobot_hardware project. The teensy dose not support WIFI.
- [esp32](https://en.wikipedia.org/wiki/ESP32) - support added in this fork. There are many variants. The RISC-V variant esp32-c3 does not support PCNT (hardware pulse counters), which is needed for encoders. So the esp32-c3 is not recommended.

## Quick start

Install essential build tools. Remove brltty package which interferes with CH340 USB serial on some esp32 boards.

    sudo apt remove brltty -y
    sudo apt install python3-venv build-essential cmake git curl -y

Install [PlatformIO CLI](https://docs.platformio.org/en/latest/core/installation/methods/installer-script.html#super-quick-macos-linux).

    curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
    python3 get-platformio.py

Add platformio to $PATH in ~/.bashrc or ~/.profile .

    echo "PATH=\"\$PATH:\$HOME/.platformio/penv/bin\"" >> $HOME/.bashrc
    source $HOME/.bashrc

Install [PlatformIO udev rules](https://docs.platformio.org/en/latest/core/installation/udev-rules.html).

    curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
    sudo service udev restart
    sudo usermod -a -G dialout $USER
    sudo usermod -a -G plugdev $USER

Clone the source.

    git clone https://github.com/hippo5329/linorobot2_hardware.git

Assume we are using the esp32tank configuration. Update the configuration file with your wifi keys and servers IP.

    linorobot2_hardware/config/custom/esp32tank_config.h

    #define AGENT_IP { 192, 168, 1, 100 }  // eg IP of the desktop computer
    #define WIFI_AP_LIST {{"WIFI_SSID", "WIFI_PASSWORD"}, {NULL}}
    #define SYSLOG_SERVER { 192, 168, 1, 100 }  // eg IP of the desktop computer
    #define LIDAR_SERVER { 192, 168, 1, 100 }  // eg IP of the desktop computer

Build and upload with USB cable connected to the esp32 board.

    cd linorobot2_hardware/firmware
    pio run -e esp32tank -t upload

You may try this on the esp32 dev board even before building the robot. The LED on the esp32 dev board may flash three times per second, that means the IMU (MPU6050) is not detected. Add the IMU, then the it will try to connect the micro ROS agent with wifi transport.

    # Run a micro-ROS agent
    ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

After it is connected to the agent, you may check the topics it published.

    ros2 topic list
    ros2 topic echo /imu/data

Please check the [Wiki](https://github.com/hippo5329/linorobot2_hardware/wiki) for configurations, guides and documents.
Questions (via [Discussions](https://github.com/hippo5329/linorobot2_hardware/discussions)),
bug reports (via [Issues](https://github.com/hippo5329/linorobot2_hardware/issues)) and
code contributions (via [Pull requests](https://github.com/hippo5329/linorobot2_hardware/pulls)) are welcome.