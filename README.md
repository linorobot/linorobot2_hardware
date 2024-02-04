# linorobot2_hardware with esp32 support

This is a fork of the [linorobot/linorobot2_hardware](https://github.com/linorobot/linorobot2_hardware) project which builds firmware for micro-controllers to control mobile robots based on micro-ROS with various sensors support. The esp32 support is added in this fork.

## Supported micro-controllers

- [teensy](https://www.pjrc.com/teensy/) - the original micro-controller supported in the linorobot/linorobot_hardware project. The teensy dose not support WIFI.
- [esp32](https://en.wikipedia.org/wiki/ESP32) - support added in this fork. There are many variants. The RISC-V variant esp32-c3 does not support PCNT (hardware pulse counters), which is needed for encoders. So the esp32-c3 is not recommended.

You will need to specify the [board variant](https://docs.platformio.org/en/latest/boards/index.html#espressif-32) in the project configuration file linorobot2_hardware/firmware/platformio.ini.

## Why esp32

The WIFI capability of the esp32 can be used to build low-cost mobile robots with navigation control under ROS2.

- remote development - develop on your desktop with powerful [PlatformIO IDE for VSCode](https://platformio.org/install/ide?install=vscode).
- remote firmware update - with ArduinoOTA.
- remote debug messages - with remote syslog.
- remote lidar server - lidar raw data is forwarded to a server to publish laser scan data.
- remote ROS2/NAV2 stack - with micro-ROS wifi transport.
- no robot computer is required - a single esp32 will serve the robot.

In traditional robot builds, there are robot computers (such as raspberry pi, jetson nono or mini PC) running the build tools and ROS2/NAV2 navigation stack. With the WIFI capability of the esp32, the robot firmware can be developed on your laptop or desktop PC. After the firmware is written to the esp32 using USB cable, the following firmware updates can be performed remotely using ArduinoOTA. The debug messages can be read using remote syslog server. Using the WIFI transport of the micro-ROS, the navigation packages NAV2 / SLAM and visualization tool RVIZ2 can be served on your laptop or desktop PC. This is a more comfortable development environment than the more restricted robot computers. No robot computer is required on the robot now. Just an esp32 micro-controller will serve. And the cost of the robot will be a little cheaper.

If you are new to esp32, you may check the [guide to esp32](https://randomnerdtutorials.com/getting-started-with-esp32/).

## Supported robot types

- 2wd - 2 wheel drive robot.
- 4wd - 4 wheel drive robot.
- mecanum - Mecanum drive robot.

The PID algorithm is used to control the motors speed.

## Supported motor drivers

- 2 Direction pins(INA, INB) and 1 PWM(ENABLE) pin - L298, L293, VNH5019, TB6612
- 1 Direction pin(INA) and 1 PWM(ENABLE) pin
- 2 PWM pins (INA, INB) - BTS7970, A4950 (<40V), DRV8833 (<10V), TB6612 with PWM pin pull-high
- ESC - brush-less motor

The 2 PWM pins driver is recommended for esp32 to reduce the number of I/O pins used.

## Supported sensors

- IMU - GY85, MPU6050, MPU9150, MPU9250, QMI8658.
- compass - HMC5883L, AK8963, AK 8975, AK09918, QMC5883L.
- encoder - PCNT (hardware pulse counter on esp32), interrupt driven
- battery - on-chip ADC, INA219
- ultrasonic - HC-SR04
- lidar - ldlidar LD19/D300

## Configuration examples

The firmware is configured with a configuration file in the linorobot2_hardware/config/custom/ directory. There is a configuration file for each robot. The configuration file contains various C macro to define the options. The following are examples which you may modify or copy to create your own.

- gendrv_config.h - 2WD, [waveshare general driver for robots](https://www.waveshare.com/general-driver-for-robots.htm) is an all-in-one esp32 board. It can save some work if you are new to esp32 and hardware stuff.
- esp32_config.h - 2WD, esp32 dev board and MPU6050.

## Quick start

Install essential build tools. Remove brltty package which interferes with CH340 USB serial on some esp32 boards.

    sudo apt remove brltty -y
    sudo apt install python3-venv build-essential cmake git curl -y

Install [PlatformIO IDE for VSCode](https://platformio.org/install/ide?install=vscode) (which will install PlatformIO CLI automatically). Or install only [PlatformIO CLI](https://docs.platformio.org/en/latest/core/installation/methods/installer-script.html#super-quick-macos-linux) with the following,

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

Assume we are using the esp32 configuration. Update the configuration file with your wifi keys and servers IP.

    linorobot2_hardware/config/custom/esp32_config.h

    #define AGENT_IP { 192, 168, 1, 100 }  // eg IP of the desktop computer
    #define WIFI_AP_LIST {{"WIFI_SSID", "WIFI_PASSWORD"}, {NULL}}
    #define SYSLOG_SERVER { 192, 168, 1, 100 }  // eg IP of the desktop computer
    #define LIDAR_SERVER { 192, 168, 1, 100 }  // eg IP of the desktop computer

Build and upload with USB cable connected to the esp32 board. You should try this on the esp32 dev board before building the robot.

    cd linorobot2_hardware/firmware
    pio run -e esp32 -t upload

The serial monitor will print the IP of esp32 (here 192.168.1.101) after connected to the wifi.

    WIFI connected
    IP address: 192.168.1.101

The LED on the esp32 dev board may flash three times per second, that means the IMU (MPU6050) is not detected. Add the IMU, then the it will try to connect to the [micro-ROS agent](https://github.com/micro-ROS/micro_ros_setup?tab=readme-ov-file#building-micro-ros-agent) with wifi transport.

    # Run a micro-ROS agent
    ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

After it is connected to the agent, you may check the topics it published.

    ros2 topic list
    ros2 topic echo /imu/data

Please check the [Wiki](https://github.com/hippo5329/linorobot2_hardware/wiki) for more details.

Questions (via [Discussions](https://github.com/hippo5329/linorobot2_hardware/discussions)),
bug reports (via [Issues](https://github.com/hippo5329/linorobot2_hardware/issues)) and
code contributions (via [Pull requests](https://github.com/hippo5329/linorobot2_hardware/pulls)) are welcome.
