# Installation

## 1. Installing ROS2 and micro-ROS in the host computer

## 1.1 ROS2 Installation

You can use the script found in [ros2me](https://github.com/linorobot/ros2me) to install ROS2 - Foxy. Take note that this only works on ROS Foxy and above.

    git clone https://github.com/linorobot/ros2me
    cd ros2me
    ./install

Once you're done with the installation download the teleoperation package:

    sudo apt install ros-foxy-teleop-twist-keyboard 

## 1.2 Micro-ROS Installation

Source your ROS2 distro and workspace:

    source /opt/ros/<your_ros_distro>/setup.bash
    cd <your_ws>
    colcon build
    source install/local_setup.bash

Download and install micro-ROS:

    cd <your_ws>
    git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
    sudo apt update && rosdep update
    rosdep install --from-path src --ignore-src -y
    colcon build
    source install/local_setup.bash

Setup micro-ROS agent:

    ros2 run micro_ros_setup create_agent_ws.sh
    ros2 run micro_ros_setup build_agent.sh
    source install/local_setup.bash

## 2. Install PlatformIO

You can check out the installation guide how to Install PlatformIO [here](https://docs.platformio.org/en/latest//core/installation.html).

## 3. UDEV Rule
Download the udev rules from Teensy's website:

    wget https://www.pjrc.com/teensy/00-teensy.rules

and copy the file to /etc/udev/rules.d :

    sudo cp 00-teensy.rules /etc/udev/rules.d/


## 4. Configure your Teensy Board
The default board used in this demo is Teensy 3.6 but you can uncomment the Teensy board you're using

    [env:teensy36]
    board = teensy36

    ; [env:teensy35]
    ; board = teensy35

    ; [env:teensy36]
    ; board = teensy36


## 5. Configure robot settings

Go to lib/config and open lino_base_config.h and uncomment the base, motor driver and IMU you want to use. For example:

    #define LINO_BASE DIFFERENTIAL_DRIVE
    #define USE_L298_DRIVER
    #define USE_GY85_IMU

## 5. Upload the firmware

Upload the firmware by running:

    cd linorobot2_prototype
    pio run --target upload

# Running the demo

## 1. Run the micro-ROS agent.

This will allow the robot to receive Twist messages to control the robot, and publish odometry and imu data.

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