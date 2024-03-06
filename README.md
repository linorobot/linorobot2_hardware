## Build status
<!-- Build Status populated by Github Actions runs -->
ROS 2 Distro | Branch | Build status
:----------: | :----: | :----------:
**Rolling** | [`rolling`](../../tree/rolling) | [![Rolling Firmware Build](../../actions/workflows/rolling-firmware-build.yml/badge.svg?branch=rolling)](../../actions/workflows/rolling-firmware-build.yml?branch=rolling)
**Humble** | [`humble`](../../tree/humble) | [![Humble Firmware Build](../../actions/workflows/humble-firmware-build.yml/badge.svg?branch=humble)](../../actions/workflows/humble-firmware-build.yml?branch=humble)
**Galactic** | [`galactic`](../../tree/galactic) | [![Galactic Firmware Build](../../actions/workflows/galactic-firmware-build.yml/badge.svg?branch=galactic)](../../actions/workflows/galactic-firmware-build.yml?branch=galactic)
**Foxy** | [`foxy`](../../tree/foxy) | [![Foxy Firmware Build](../../actions/workflows/foxy-firmware-build.yml/badge.svg?branch=foxy)](../../actions/workflows/foxy-firmware-build.yml?branch=foxy)

## Installation
All software mentioned in this guide must be installed on the robot computer.

### 1. ROS2 and linorobot2 installation
It is assumed that you already have ROS2 and linorobot2 package installed. If you haven't, go to [linorobot2](https://github.com/linorobot/linorobot2) package for installation guide.

### 2. Download linorobot2_hardware

    cd $HOME
    git clone https://github.com/linorobot/linorobot2_hardware -b $ROS_DISTRO

### 3. Install PlatformIO
Download and install platformio. [Platformio](https://platformio.org/) allows you to develop, configure, and upload the firmware without the Arduino IDE. This means that you can upload the firmware remotely which is ideal on headless setup especially when all components have already been fixed. 
    
    curl -fsSL -o get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
    python3 get-platformio.py
    
Add platformio to your $PATH:

    echo "PATH=\"\$PATH:\$HOME/.platformio/penv/bin\"" >> $HOME/.bashrc
    source $HOME/.bashrc

### 4. UDEV Rule
Download the udev rules from Teensy's website:

    wget https://www.pjrc.com/teensy/00-teensy.rules

and copy the file to /etc/udev/rules.d :

    sudo cp 00-teensy.rules /etc/udev/rules.d/

### 5. Install Screen Terminal

    sudo apt install screen

## Building the robot

### 1. Robot orientation
Robot Orientation:

-------------FRONT-------------

WHEEL1 WHEEL2 (2WD)

WHEEL3 WHEEL4 (4WD)

--------------BACK--------------

If you're building a 2 wheel drive robot, assign `MOTOR1` and `MOTOR2` to the left and right motors respectively.

For mecanum robots, follow the wheels' orientation below.

![mecanum_wheels_orientation](docs/mecanum_wheels_orientation.png)

### 2. Motor Drivers

Supported Motor Drivers:

- **GENERIC_2_IN_MOTOR_DRIVER** - Motor drivers that have EN (pwm) pin, and 2 direction pins (usually DIRA, DIRB pins). Example: L298 Breakout boards.

- **GENERIC_1_IN_MOTOR_DRIVER** - Motor drivers that have EN (pwm) pin, and 1 direction pin (usual DIR pin). These drivers usually have logic gates included to lessen the pins required in controlling the driver. Example: Pololu MC33926 Motor Driver Shield.

- **BTS7960_MOTOR_DRIVER** - BTS7960 motor driver.

- **ESC_MOTOR_DRIVER** - Bi-directional (forward/reverse) electronic speed controllers.

The motor drivers are configurable from the config file explained in the later part of this document.

### 3. Inertial Measurement Unit (IMU)

Supported IMUs:

- **GY-85**
- **MPU6050**
- **MPU9150**
- **MPU9250**

### 4. Connection Diagram
Below are connection diagrams you can follow for each supported motor driver and IMU. For simplicity, only one motor connection is provided but the same diagram can be used to connect the rest of the motors. You are free to decide which microcontroller pin to use just ensure that the following are met:

- Reserve SCL0 and SDA0 (pins 18 and 19 on Teensy boards) for IMU.

- When connecting the motor driver's EN/PWM pin, ensure that the microcontroller pin used is PWM enabled. You can check out PJRC's [pinout page](https://www.pjrc.com/teensy/pinout.html) for more info.

Alternatively, you can also use the pre-defined pin assignments in lino_base_config.h. Teensy 3.x and 4.x have different mapping of PWM pins, read the notes beside each pin assignment in [lino_base_config.h](https://github.com/linorobot/linorobot2_hardware/blob/master/config/lino_base_config.h#L112) carefully to avoid connecting your driver's PWM pin to a non PWM pin on Teensy. 

All diagrams below are based on Teensy 4.0 microcontroller and GY85 IMU. Click the images for higher resolution.

#### 4.1 GENERIC 2 IN

![generic_2_in_connection](docs/generic_2_in_connection.png)

#### 4.2 GENERIC 1 IN

![generic_1_in_connection](docs/generic_1_in_connection.png)

#### 4.3 BTS7960

![bts7960_connection](docs/bts7960_connection.png)

#### 4.4 IMU

![imu_connection](docs/imu_connection.png)

Take note of the IMU's correct orientation when mounted on the robot. Ensure that the IMU's axes are facing the correct direction:

- **X** - Front
- **Y** - Left
- **Z** - Up

#### 4.5 System Diagram
Reference designs you can follow in building your robot.

A minimal setup with a 5V powered robot computer.
![minimal_setup](docs/minimal_setup.png)

A more advanced setup with a 19V powered computer and USB hub connected to sensors.
![advanced_setup](docs/advanced_setup.png)

For bigger robots, you can add an emergency switch in between the motor drivers' power supply and motor drivers.

## Setting up the firmware
### 1. Robot Settings
Go to the config folder and open lino_base_config.h. Uncomment the base, motor driver and IMU you want to use for your robot. For example:

    #define LINO_BASE DIFFERENTIAL_DRIVE
    #define USE_GENERIC_2_IN_MOTOR_DRIVER
    #define USE_GY85_IMU

Constants' Meaning:

*ROBOT TYPE (LINO_BASE)*
- **DIFFERENTIAL_DRIVE** - 2 wheel drive or tracked robots w/ 2 motors.

- **SKID_STEER** - 4 wheel drive robots.

- **MECANUM** - 4 wheel drive robots using mecanum wheels.

*MOTOR DRIVERS*
- **USE_GENERIC_2_IN_MOTOR_DRIVER** - Motor drivers that have EN (pwm) pin, and 2 direction pins (usually DIRA, DIRB pins).

- **USE_GENERIC_1_IN_MOTOR_DRIVER** - Motor drivers that have EN (pwm) pin, and 1 direction pin (usual DIR pin). These drivers usually have logic gates included to lessen the pins required in controlling the driver.

- **USE_BTS7960_MOTOR_DRIVER** - BTS7960 motor driver.

- **USE_ESC_MOTOR_DRIVER** - Bi-directional (forward/reverse) electronic speed controllers.

*INERTIAL MEASUREMENT UNIT (IMU)*
- **USE_GY85_IMU** - GY-85 IMUs.

- **USE_MPU6050_IMU** - MPU6060 IMUs.

- **USE_MPU9150_IMU** - MPU9150 IMUs.

- **USE_MPU9250_IMU** - MPU9250 IMUs.

Next, fill in the robot settings accordingly:

    #define K_P 0.6
    #define K_I 0.8
    #define K_D 0.5

    #define MOTOR_MAX_RPM 100             
    #define MAX_RPM_RATIO 0.85          
    #define MOTOR_OPERATING_VOLTAGE 24
    #define MOTOR_POWER_MAX_VOLTAGE 12
    #define MOTOR_POWER_MEASURED_VOLTAGE 11.7

    #define COUNTS_PER_REV1 2200    
    #define COUNTS_PER_REV2 2200      
    #define COUNTS_PER_REV3 2200      
    #define COUNTS_PER_REV4 2200      
  
    #define WHEEL_DIAMETER 0.09  
    #define LR_WHEELS_DISTANCE 0.2  

    #define PWM_BITS 10
    #define PWM_FREQUENCY 20000

Constants' Meaning:

- **K_P, K_I, K_D** - [PID](https://en.wikipedia.org/wiki/PID_controller) constants used to translate the robot's target velocity to motor speed. These values would likely work on your build, change these only if you experience jittery motions from the robot or you'd want to fine-tune it further.

- **MOTOR_MAX_RPM** - Motor's maximum number of rotations it can do in a minute specified by the manufacturer.

- **MAX_RPM_RATIO** - Percentage of the motor's maximum RPM that the robot is allowed to move. This parameter ensures that the user-defined velocity will not be more than or equal the motor's max RPM, allowing the PID to have ample space to add/subtract RPM values to reach the target velocity. For instance, if your motor's maximum velocity is 0.5 m/s with `MAX_RPM_RATIO` set to 0.85, and you asked the robot to move at 0.5 m/s, the robot's maximum velocity will be capped at 0.425 m/s (0.85 * 0.5m/s). You can set this parameter to 1.0 if your wheels can spin way more than your operational speed.

    Wheel velocity can be computed as:  MAX_WHEEL_VELOCITY = (`MOTOR_MAX_RPM` / 60.0) * PI * `WHEEL_DIAMETER` 

- **MOTOR_OPERATING_VOLTAGE** - Motor's operating voltage specified by the manufacturer (usually 5V/6V, 12V, 24V, 48V). This parameter is used to calculate the motor encoder's `COUNTS_PER_REV` constant during calibration and actual maximum RPM of the motors. For instance, a robot with `MOTOR_OPERATING_VOLTAGE` of 24V with a `MOTOR_POWER_MAX_VOLTAGE` of 12V, will only have half of the manufacturer's specified maximum RPM ((`MOTOR_POWER_MAX_VOLTAGE` / `MOTOR_OPERATING_VOLTAGE`) * `MOTOR_MAX_RPM`). 

- **MOTOR_POWER_MAX_VOLTAGE** - Maximum voltage of the motor's power source. This parameter is used to calculate the actual maximum RPM of the motors.

- **MOTOR_POWER_MEASURED_VOLTAGE** - Measured voltage of the motor's power source. If you don't have a multimeter, it's best to fully charge your battery and set this parameter to your motor's operating voltage (`MOTOR_OPERATING_VOLTAGE`). This parameter is used to calculate the motor encoder's `COUNTS_PER_REV` constant. You can ignore this if you're using the manufacturer's specified counts per rev.

- **COUNTS_PER_REVX** - The total number of pulses the encoder has to read to be considered as one revolution. You can either use the manufacturer's specification or the calibrated value in the next step. If you're planning to use the calibrated value, ensure that you have defined the correct values for `MOTOR_OPERATING_VOLTAGE` and `MOTOR_POWER_MEASURED_VOLTAGE`.

- **WHEEL_DIAMETER** - Diameter of the wheels in meters.

- **LR_WHEELS_DISTANCE** - The distance between the center of left and right wheels in meters.

- **PWM_BITS** - Number of bits in generating the PWM signal. You can use the default value if you're unsure what to put here. More info [here](https://www.pjrc.com/teensy/td_pulse.html).

- **PWM_FREQUENCY** - Frequency of the PWM signals used to control the motor drivers. You can use the default value if you're unsure what to put here. More info [here](https://www.pjrc.com/teensy/td_pulse.html).

### 2. Hardware Pin Assignments
Only modify the pin assignments under the motor driver constant that you are using ie. `#ifdef USE_GENERIC_2_IN_MOTOR_DRIVER`. You can check out PJRC's [pinout page](https://www.pjrc.com/teensy/pinout.html) for each board's pin layout.

The pin assignments found in lino_base_config.h are based on Linorobot's PCB board. You can wire up your electronic components based on the default pin assignments but you're also free to modify it depending on your setup. Just ensure that you're connecting MOTORX_PWM pins to a PWM enabled pin on the microcontroller and reserve SCL and SDA pins for the IMU, and pin 13 (built-in LED) for debugging.

    // INVERT ENCODER COUNTS
    #define MOTOR1_ENCODER_INV false 
    #define MOTOR2_ENCODER_INV false 
    #define MOTOR3_ENCODER_INV false 
    #define MOTOR4_ENCODER_INV false 

    // INVERT MOTOR DIRECTIONS
    #define MOTOR1_INV false
    #define MOTOR2_INV false
    #define MOTOR3_INV false
    #define MOTOR4_INV false

    // ENCODER PINS
    #define MOTOR1_ENCODER_A 14
    #define MOTOR1_ENCODER_B 15 

    #define MOTOR2_ENCODER_A 11
    #define MOTOR2_ENCODER_B 12 

    #define MOTOR3_ENCODER_A 17
    #define MOTOR3_ENCODER_B 16 

    #define MOTOR4_ENCODER_A 9
    #define MOTOR4_ENCODER_B 10

    // MOTOR PINS
    #ifdef USE_GENERIC_2_IN_MOTOR_DRIVER
        #define MOTOR1_PWM 21 //Pin no 21 is not a PWM pin on Teensy 4.x, you can swap it with pin no 1 instead.
        #define MOTOR1_IN_A 20
        #define MOTOR1_IN_B 1 

        #define MOTOR2_PWM 5
        #define MOTOR2_IN_A 6
        #define MOTOR2_IN_B 8

        #define MOTOR3_PWM 22
        #define MOTOR3_IN_A 23
        #define MOTOR3_IN_B 0

        #define MOTOR4_PWM 4
        #define MOTOR4_IN_A 3
        #define MOTOR4_IN_B 2

        #define PWM_MAX pow(2, PWM_BITS) - 1
        #define PWM_MIN -PWM_MAX
    #endif  

Constants' Meaning:

- **MOTORX_ENCODER_A** - Microcontroller pin that is connected to the first read pin of the motor encoder. This pin is usually labelled as A pin on the motor encoder board.

- **MOTORX_ENCODER_B** - Microcontroller pin that is connected to the second read pin of the motor encoder. This pin is usually labelled as B pin on the motor encoder board.

- **MOTORX_ENCODER_INV** - Flag used to change the sign of the encoder value. More on that later.

- **MOTORX_PWM** - Microcontroller pin that is connected to the PWM pin of the motor driver. This pin is usually labelled as EN or ENABLE pin on the motor driver board. 

- **MOTORX_IN_A** - Microcontroller pin that is connected to one of the motor driver's direction pins. This pin is usually labelled as DIRA or DIR1 pin on the motor driver board. On BTS7960 driver, this is one of the two PWM pins connected to the driver (RPWM/LPWM).

- **MOTORX_IN_B** - Microcontroller pin that is connected to one of the motor driver's direction pins. This pin is usually labelled as DIRB or DIR2 pin on the motor driver board. On BTS7960 driver, this is one of the two PWM pins connected to the driver (RPWM/LPWM).

- **MOTORX_INV** - Flag used to invert the direction of the motor. More on that later.

### 3. WIFI related settings

- **USE_WIFI_TRANSPORT** - use micro-ROS wifi transport.

- **AGENT_IP** - micro-ROS agent IP. eg. { 192, 168, 1, 100 }

- **AGENT_PORT** - micro-ROS agent port. default 8888

- **WIFI_AP_LIST** - Enable WiFi with null terminated list of multiple APs SSID and password. eg. {{"WIFI_SSID1", "WIFI_PASSWORD1"}, {"WIFI_SSID2", "WIFI_PASSWORD2"}, {NULL}} . The AP with strongest signal will be used. When wifi signal is too low, the current AP will be disconnected and reconnect the AP with the strongest signal.

- **USE_ARDUINO_OTA** - Arduino OTA up load protocol support

- **USE_SYSLOG** - logging to remote syslog server.

- **SYSLOG_SERVER** - syslog server name or IP.

- **SYSLOG_PORT** - syslog server udp port. default 514

- **DEVICE_HOSTNAME** - my device name to syslog. default "linorobot2"

- **APP_NAME** - my app name to syslog. default "hardware"

- **USE_LIDAR_UDP** - push Lidar data to UDP server, which will decode the data and publish laser scan message.

- **LIDAR_RXD** - RXD pin for serial data from Lidar

- **LIDAR_BAUDRATE**

- **LIDAR_SERVER** - Lidar server IP address, eg. { 192, 168, 1, 100 }

- **LIDAR_PORT** - Lidar server UDP port, eg. 8889

### 4. Optional settings

- **BAUDRATE** - serial baudrate. default 115200

- **NODE_NAME** - ROS2 node name. default "linorobot_base_node"

- **TOPIC_PREFIX** - Namespace prefix to topic, eg "turtle1/". Useful when there are multiple robots running.

- **USE_SHORT_BRAKE** - Short brake for shorter distance to stop, only for generic_2 BT6612 and BTS7960 like motor drivers

- **BOARD_INIT** - early board specific setup

- **BOARD_INIT_LATE** - late board specific setup

## Calibration
Before proceeding, **ensure that your robot is elevated and the wheels aren't touching the ground**. 
5.1
### 1. Motor Check
Go to calibration folder and upload the firmware:

    cd linorobot2_hardware/calibration
    pio run --target upload -e <your_teensy_board>

Available Teensy boards:
- teensy31
- teensy35
- teensy36
- teensy40
- teensy41

Some Linux machines might encounter a problem related to libusb. If so, install libusb-dev:

    sudo apt install libusb-dev

Start spinning the motors by running:
    
    screen /dev/ttyACM0

On the terminal type `spin` and press the enter key.

The wheels will spin one by one for 10 seconds from Motor1 to Motor4. Check if each wheel's direction is spinning **forward** and take note of the motors that are spinning in the opposite direction. Set MOTORX_INV constant in [lino_base_config.h](https://github.com/linorobot/linorobot2_hardware/blob/master/config/lino_base_config.h#L71-L74) to `true` to invert the motor's direction. Reupload the calibration firmware once you're done. Press `Ctrl` + `a` + `d` to exit the screen terminal.

    cd linorobot2_hardware/calibration
    pio run --target upload -e <your_teensy_board>

### 2. Encoder Check

Open your terminal and run:

    screen /dev/ttyACM0

Type `sample` and press the enter key. Verify if all the wheels are spinning **forward**. Redo the previous step if there are motors still spinning in the opposite direction.

You'll see a summary of the total encoder readings and counts per revolution after the motors have been sampled. If you see any negative number in the MOTOR ENCODER READINGS section, invert the encoder pin by setting `MOTORX_ENCODER_INV` in [lino_base_config.h](https://github.com/linorobot/linorobot2_hardware/blob/master/config/lino_base_config.h#L65-L68) to `true`. Reupload the calibration firmware to check if the encoder pins have been reconfigured properly:

    cd linorobot2_hardware/calibration
    pio run --target upload -e <your_teensy_board>
    screen /dev/ttyACM0

Type `sample` and press the enter key. Verify if all encoder values are now **positive**. Redo this step if you missed out any.

### 3. Counts Per Revolution

On the previous instruction where you check the encoder reads for each motor, you'll see that there's also COUNTS PER REVOLUTION values printed on the screen. If you have defined `MOTOR_OPERATING_VOLTAGE` and `MOTOR_POWER_MEASURED_VOLTAGE`, you can assign these values to `COUNTS_PER_REVX` constants in [lino_base_config.h](https://github.com/linorobot/linorobot2_hardware/blob/master/config/lino_base_config.h#L55-L58) to have a more accurate model of the encoder.

## Upload the firmware
Ensure that the robot pass all the requirements before uploading the firmware:

- Defined the correct motor rpm.
- Motors' IDs are correct.
- Motors spin in the right direction.
- Encoders' signs are correct.
- Defined the correct encoder's COUNTS_PER_REV.
- Defined the correct robot type.
- Defined the correct motor driver.
- Defined the correct IMU.
- Defined the correct wheel diameter.
- Defined the correct distance between wheels.

Run:

    cd linorobot2_hardware/firmware
    pio run --target upload -e <your_teensy_board>

## Testing the robot

### 1. Run the micro-ROS agent.

This will allow the robot to receive Twist messages to control the robot, and publish odometry and IMU data straight from the microcontroller. Compared to Linorobot's ROS1 version, the odometry and IMU data published from the microcontroller use standard ROS2 messages and do not require any relay nodes to reconstruct the data to complete [sensor_msgs/Imu](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html) and [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) messages.

Run the agent:

    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0

### 2. Drive around

Run teleop_twist_keyboard package and follow the instructions on the terminal on how to drive the robot:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard 

### 3. Check the topics

Check if the odom and IMU data are published:

    ros2 topic list

Now you should see the following topics:

    /cmd_vel
    /imu/data
    /odom/unfiltered
    /parameter_events
    /rosout

Echo odometry data:

    ros2 topic echo /odom/unfiltered

Echo IMU data:

    ros2 topic echo /imu/data

## USB ports on esp32-s2 and esp32-s3

The esp32-s2 and esp32-s3 have onchip USB CDC devices, which will be faster than USB uart bridge like CP2102. They will become default serial port /dev/ttyACM0 when enabled with the ARDUINO_USB_CDC_ON_BOOT build flag. But the serial driver of esp32-s2 has issue with micro-ROS serial transport, though there is no problem with wifi transport. There may be two USB ports on the dev boards, one marked as "USB" (the onchip CDC) and the other marked as "COM" (the usb uart bridge). You should use the one marked as "USB" for esp32-s3. And use "COM" for esp32-s2. If there is only one USB port, you should check the schematic to find out which is connected.

esp32-s3

- USB - /dev/ttyACM0 default, enabled with the ARDUINO_USB_CDC_ON_BOOT build flag
- COM - /dev/ttyACM0

esp32-s2

- USB - /dev/ttyACM0 no use, enabled with the ARDUINO_USB_CDC_ON_BOOT build flag, but not stable with micro-ROS.
- COM - /dev/ttyUSB0 default

Build flag,

    build_flags =
        -I ../config
        -D ARDUINO_USB_CDC_ON_BOOT
        -D USE_ESP32S3_CONFIG

## Use esp32 with micro-ROS wifi transport, OTA, syslog and Lidar UDP transport

The WIFI capability of the esp32 can be used to build low-cost mobile robots with navigation control under ROS2.

- remote development - develop on your desktop with powerful [PlatformIO IDE for VSCode](https://platformio.org/install/ide?install=vscode).
- remote firmware update - with ArduinoOTA.
- remote debug messages - with remote syslog.
- remote lidar server - lidar raw data is forwarded to a server to publish laser scan data.
- remote ROS2/NAV2 stack - with micro-ROS wifi transport.
- no robot computer is required - a single esp32 will serve the robot.

In traditional robot builds, there are robot computers (such as raspberry pi, jetson nono or mini PC) running the build tools and ROS2/NAV2 navigation stack. With the WIFI capability of the esp32, the robot firmware can be developed on your laptop or desktop PC. After the firmware is written to the esp32 using USB cable, the following firmware updates can be performed remotely using ArduinoOTA. The debug messages can be read using remote syslog server. Using the WIFI transport of the micro-ROS, the navigation packages NAV2 / SLAM and visualization tool RVIZ2 can be served on your laptop or desktop PC. This is a more comfortable development environment than the more restricted robot computers. No robot computer is required on the robot now. Just an esp32 micro-controller will serve. And the cost of the robot will be a little cheaper.

If you are new to esp32, you may check the [guide to esp32](https://randomnerdtutorials.com/getting-started-with-esp32/).

### 1. Start the micro ros wifi transport agent

    ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

### 2. Enable remote syslog server - Optional

Edit syslog server config. Add the following to the end.

    sudo nano /etc/rsyslog.conf

    # provides UDP syslog reception
    module(load="imudp")
    input(type="imudp" port="514")

    $template Incoming-logs,"/var/log/%HOSTNAME%/%PROGRAMNAME%.log"
    *.* ?Incoming-logs

Restart the syslog service

    sudo systemctl restart rsyslog

### 3. Modify custom configuration

The following is the project ini, pins wiring and custom configuration changes. The first build and upload will use USB serial port.

Change in ../config/custom/myrobot_config.h

    #define USE_WIFI_TRANSPORT  // use micro ros wifi transport
    #define AGENT_IP { 192, 168, 1, 100 }  // eg your desktop IP addres
    #define AGENT_PORT 8888
    #define WIFI_AP_LIST {{"WIFI_SSID", "WIFI_PASSWORD"}, {NULL}}
    #define USE_ARDUINO_OTA
    #define USE_SYSLOG
    #define SYSLOG_SERVER { 192, 168, 1, 100 }  // eg your desktop IP addres
    #define SYSLOG_PORT 514
    #define DEVICE_HOSTNAME "myrobot"
    #define APP_NAME "hardware"

Change ini for 1st run, upoad with serial port.

    [env:myrobot]
    upload_port = /dev/ttyUSB0
    upload_protocol = esptool
    board_microros_transport = wifi

### 4. Build and upload

    pio run -e myrobot -t upload
    pio device monitor -e myrobot -p <serial_port> -b <baudrate>

The serial will print the IP of esp32 after connected to wifi.

    WIFI connected
    IP address: 192.168.1.101

The IP address is also displayed in the micro ros wifi agent messge.

    [1686587945.807239] info     | UDPv4AgentLinux.cpp | init                     | running...             | port: 8888
    [1686587945.807411] info     | Root.cpp           | set_verbose_level        | logger setup           | verbose_level: 4
    [1686588315.310850] info     | Root.cpp           | create_client            | create                 | client_key: 0x0C7BC5A9, session_id: 0x81
    [1686588315.310892] info     | SessionManager.hpp | establish_session        | session established    | client_key: 0x0C7BC5A9, address: 192.168.1.101:47138
    [1686588315.327314] info     | ProxyClient.cpp    | create_participant       | participant created    | client_key: 0x0C7BC5A9, participant_id: 0x000(1)
    [1686588315.332840] info     | ProxyClient.cpp    | create_topic             | topic created          | client_key: 0x0C7BC5A9, topic_id: 0x000(2), participant_

And check the syslog.

    sudo cat /var/log/myrobot/hardware.log

    2023-06-13T14:34:41.978640-07:00 myrobot hardware initWifis ssid SSID rssi -44 ip 192.168.1.101
    2023-06-13T14:34:44.042429-07:00 myrobot hardware setup Ready 9344
    2023-06-13T14:34:44.548896-07:00 myrobot hardware loop agent available 9852
    2023-06-13T14:34:44.549019-07:00 myrobot hardware createEntities 9852

### 5. OTA upload - Optional

After the esp32 connected to wifi. Read the esp32 IP address. Modify the project configuration ini with the esp32 IP address. Then esp32 can be uploaded remotely with OTA.

Change ini for the second run to upload with OTA.

    [env:myrobot]
    ; upload_port = /dev/ttyUSB0
    ; upload_protocol = esptool
    upload_protocol = espota
    upload_port = 192.168.1.101   <- replace with the esp32 IP address
    board_microros_transport = wifi

Upload

    Configuring upload protocol...
    AVAILABLE: cmsis-dap, esp-bridge, esp-prog, espota, esptool, iot-bus-jtag, jlink, minimodule, olimex-arm-usb-ocd, olimex-arm-usb-ocd-h, olimex-arm-usb-tiny-h, olimex-jtag-tiny, tumpa
    CURRENT: upload_protocol = espota
    Uploading .pio/build/myrobot/firmware.bin
    14:34:22 [DEBUG]: Options: {'esp_ip': '192.168.1.101', 'host_ip': '0.0.0.0', 'esp_port': 3232, 'host_port': 54252, 'auth': '', 'image': '.pio/build/myrobot/firmware.bin', 'spiffs': False, 'debug': True, 'progress': True, 'timeout': 10}
    14:34:22 [INFO]: Starting on 0.0.0.0:54252
    14:34:22 [INFO]: Upload size: 951408
    Sending invitation to 192.168.1.101
    14:34:22 [INFO]: Waiting for device...

    Uploading: [                                                            ] 0%
    Uploading: [=                                                           ] 0%
    ...
    Uploading: [============================================================] 99%
    Uploading: [============================================================] 100% Done...

    14:34:33 [INFO]: Waiting for result...
    14:34:34 [INFO]: Result: OK
    14:34:34 [INFO]: Success
    ========================= [SUCCESS] Took 20.84 seconds =========================

### 6. Setup the Lidar UDP transport - Optional

The lidar data can be pushed to a UDP  server, which will decode the data and publish laser scan message. The UDP client on the esp32 should work with most Lidar. For the server side, only LdLidar is supported. Only LD19 launch file is tested. Conenct VCC and GND to Lidar. Connect Lidar TXD wire to LIDAR_RXD pin of esp32.

change in ../config/custom/myrobot_config.h

    #define USE_LIDAR_UDP
    #define LIDAR_RXD 14 // you may use any available input pin
    // #define LIDAR_PWM 15  // do not use, the PWM control loop is not implememted yet
    #define LIDAR_SERIAL 1 // uart number, 1 or 2
    #define LIDAR_BAUDRATE 230400 // the Lidar serial buadrate
    #define LIDAR_SERVER { 192, 168, 1, 100 }  // eg your desktop IP addres
    #define LIDAR_PORT 8889 // the UDP port on server

Build and launch the UDP server

    cd ~
    mkdir -p ldlidar_ros2_ws/src
    cd ldlidar_ros2_ws/src
    git clone  https://github.com/hippo5329/ldlidar_stl_ros2.git
    cd ..
    colcon build
    source install/setup.bash
    ros2 launch ldlidar_stl_ros2 ld19_udp_server.launch.py

## URDF
Once the hardware is done, you can go back to [linorobot2](https://github.com/linorobot/linorobot2#urdf) package and start defining the robot's URDF.

## Troubleshooting Guide

### 1. One of my motor isn't spinning.
- Check if the motors are powered.
- Check if you have bad wiring.
- Check if you have misconfigured the motor's pin assignment in lino_base_config.h.
- Check if you uncommented the correct motor driver (ie. `USE_GENERIC_2_IN_MOTOR_DRIVER`)
- Check if you assigned the motor driver pins under the correct motor driver constant. For instance, if you uncommented `USE_GENERIC_2_IN_MOTOR_DRIVER`, all the pins you assigned must be inside the `ifdef USE_GENERIC_2_IN_MOTOR_DRIVER` macro.

### 2. Wrong wheel is spinning during calibration process
- Check if the motor drivers have been connected to the correct microcontroller pin.
- Check if you have misconfigured the motor's pin assignment in lino_base_config.h.

### 3 One of my encoders has no reading (0 value).
- Check if the encoders are powered.
- Check if you have bad wiring.
- Check if you have misconfigured the encoder's pin assignment in lino_base_config.h.

### 4. The wheels only spin in one direction
- Check if the Teensy's GND pin is connected to the motor driver's GND pin.

### 5. The motor doesn't change it's direction after setting the INV to true.
- Check if the Teensy's GND pin is connected to the motor driver's GND pin.

### 6. Nothing's printing when I run the screen app.
- Check if you're passing the correct serial port. Run:

        ls /dev/ttyACM*
    
    and ensure that the available serial port matches the port you're passing to the screen app.

- Check if you forgot to [copy the udev rule](https://github.com/linorobot/linorobot2_hardware#3-udev-rule):

        ls /etc/udev/rules.d/00-teensy.rules 

    Remember to restart your computer if you just copied the udev rule.

### 7. The firmware was uploaded but nothing's happening.
- Check if you're assigning the correct Teensy board when uploading the firmware. If you're unsure which Teensy board you're using, take a look at the label on the biggest chip found in your Teensy board and compare it with the boards shown on PJRC's [website](https://www.pjrc.com/teensy/).

### 8. The robot's forward motion is not straight
- This happens when the target velocity is more than or equal the motor's RPM (usually happens on low RPM motors). To fix this, set the `MAX_RPM_RATIO` lower to allow the PID controller to compensate for errors.

### 9. The robot rotates after braking
- This happens due to the same reason as 7. When the motor hits its maximum rpm and fails to reach the target velocity, the PID controller's error continously increases. The abrupt turning motion is due to the PID controller's attempt to further compensate the accumulated error. To fix this, set the `MAX_RPM_RATIO` lower to allow the PID controller to compensate for errors while moving to avoid huge accumulative errors when the robot stops.

## Developers
#### Adding firmware compilation tests for a new ROS distro
To add a new distro to the CI tests, modify the `rolling` (default) branch. Inside of `.github/workflows`, duplicate an existing distro workflow YAML file. For example, to add ROS2 Iron support, one could copy `humble-firmware-build.yml` to `iron-firmware-build.yml`. Assuming that an `iron` branch exists (if not one could create one using the `humble` branch as a base and modify as necessary), inside of `iron-firmware-build.yml`, rename all instances of the word `humble` with `iron`. It would be as simple as using 'find and replace' in many IDEs. Commit these changes to a feature branch, create a PR to merge into the `rolling` branch, and then backport the PR to other branches. It is only necessary to have `iron-firmware-build.yml` on the `rolling` and `iron` branch, however it may be simpler to keep the branches in sync by having every workflow file on all branches.

Lastly, the new branch must be added to the CI table written in Markdown at the top of README.md that displays the status of each branch using badges. This table is organized with the most current ROS2 branch at the top, which is always `rolling`, and then in descending chronological order. Adding a new distro can be done by copying an existing row of the table, pasting in the appropriate position, and changing the titles and branch names in the relative paths. 
