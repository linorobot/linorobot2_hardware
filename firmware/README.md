# Installation

## 1. Installing ROS2 and micro-ROS in the host computer

## 1.1 ROS2 Installation

You can use the script found in [ros2me](https://github.com/linorobot/ros2me) to install ROS2 - Foxy. Take note that this project only works on ROS Foxy and above.

    git clone https://github.com/linorobot/ros2me
    cd ros2me
    ./install

You'll also need the teleoperation package to control the robot manually. Install ROS2's teleop_twist_keyboard package:

    sudo apt install ros-foxy-teleop-twist-keyboard 

## 1.2 micro-ROS Installation

Source your ROS2 distro and workspace:

    source /opt/ros/<your_ros_distro>/setup.bash
    cd <your_ws>
    colcon build
    source install/local_setup.bash

Download and install micro-ROS:

    cd <your_ws>
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
    sudo apt install python3-vcstool
    sudo apt update && rosdep update
    rosdep install --from-path src --ignore-src -y
    colcon build
    source install/local_setup.bash

Setup micro-ROS agent:

    ros2 run micro_ros_setup create_agent_ws.sh
    ros2 run micro_ros_setup build_agent.sh
    source install/local_setup.bash

## 2. Install PlatformIO
Download and install platformio:
    
    python3 -c "$(curl -fsSL https://raw.githubusercontent.com/platformio/platformio/master/scripts/get-platformio.py)"

Add platformio to your $PATH:

    echo "PATH=\"\$PATH:\$HOME/.platformio/penv/bin\"" >> ~/.bashrc


## 3. UDEV Rule
Download the udev rules from Teensy's website:

    wget https://www.pjrc.com/teensy/00-teensy.rules

and copy the file to /etc/udev/rules.d :

    sudo cp 00-teensy.rules /etc/udev/rules.d/


## 4. Configure your Teensy Board
The default board used in this demo is Teensy 3.1/3.2 but you can uncomment the Teensy board you're using

    [env:teensy36]
    board = teensy36

    ; [env:teensy35]
    ; board = teensy35

    ; [env:teensy36]
    ; board = teensy36


## 5. Configure robot settings

Go to lib/config folder and open lino_base_config.h. Uncomment the base, motor driver and IMU you want to use for your robot. For example:

    #define LINO_BASE DIFFERENTIAL_DRIVE
    #define USE_GENERIC_2_IN_MOTOR_DRIVER
    #define USE_GY85_IMU

Next, fill the robot settings accordingly:

    #define K_P 0.6 // P constant
    #define K_I 0.8 // I constant
    #define K_D 0.5 // D constant

    //define your robot' specs here
    #define MAX_RPM 100               // motor's maximum RPM
    #define COUNTS_PER_REV 2200       // wheel encoder's no of ticks per rev
    #define WHEEL_DIAMETER 0.09       // wheel's diameter in meters
    #define LR_WHEELS_DISTANCE 0.2    // distance between left and right wheels
    #define FR_WHEELS_DISTANCE 0.30   // distance between front and rear wheels. Ignore this if you're on 2WD
    #define PWM_BITS 8                // PWM Resolution of the microcontroller

## 5. Upload the firmware

Upload the firmware by running:

    cd linorobot2_prototype
    pio run --target upload

* Some Linux machines might encounter a problem related to libusb. If so, install libusb-dev:

    sudo apt install libusb-dev

# Running the demo

## 1. Run the micro-ROS agent.

This will allow the robot to receive Twist messages to control the robot, and publish odometry and imu data straight from the microcontroller. Compared to Linorobot's ROS1 version, the odometry and IMU data published from the microcontroller uses standard ROS2 messages and doesn't require any relay nodes to reconstruct the data to full fledge [sensor_msgs/Imu](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html) and [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html) messages.

Run the agent:

    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0

## 2. Drive around

Run teleop_twist_keyboard package and follow the instructions on the terminal on how to drive the robot:

    ros2 run teleop_twist_keyboard teleop_twist_keyboard 

## 3. Check the topics

Check if the odom and IMU data are published:

    ros2 topic list

Now you should see the following topics:

    /cmd_vel
    /imu/data
    /odom
    /parameter_events
    /rosout

You can subscribe to any of the topics by running:

    ros2 topic echo odom