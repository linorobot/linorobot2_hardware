// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <nav_msgs/msg/odometry.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/bool.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>

#include "config.h"
#include "syslog.h"
#include "motor.h"
#include "kinematics.h"
#include "pid.h"
#include "odometry.h"
#include "imu.h"
#include "mag.h"
#define ENCODER_USE_INTERRUPTS
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"
#include "fake_wheel.h"
#ifdef USE_FAKE_LD19
#include "fake_ld19.h"
#endif
#include "battery.h"
#include "range.h"
#include "lidar.h"
#include "wifis.h"
#include "ota.h"

#ifdef MICRO_ROS_TRANSPORT_ARDUINO_WIFI
// remove wifi initialization code from wifi transport
static inline void set_microros_net_transports(IPAddress agent_ip, uint16_t agent_port)
{
    static struct micro_ros_agent_locator locator;
    locator.address = agent_ip;
    locator.port = agent_port;

    rmw_uros_set_custom_transport(
        false,
        (void *) &locator,
        platformio_transport_open,
        platformio_transport_close,
        platformio_transport_write,
        platformio_transport_read
    );
}
#endif

#ifndef NODE_NAME
#define NODE_NAME "linorobot_base_node"
#endif
#ifndef TOPIC_PREFIX
#define TOPIC_PREFIX
#endif
#ifndef CONTROL_TIMER
#define CONTROL_TIMER 20 // 50Hz
#endif
#ifndef BATTERY_TIMER
#define BATTERY_TIMER 2000 // 2 sec
#endif
#ifndef RANGE_TIMER
#define RANGE_TIMER 100 // 10Hz
#endif

#ifndef RCCHECK
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop();}}
#endif
#ifndef RCSOFTCHECK
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#endif
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)

// mag.h falls back to FakeMAG and defines USE_FAKE_MAG whenever no magnetometer
// chip is configured, and the topic is then left out entirely. But fake wheel
// mode synthesises a real field from the simulated heading, hard-iron bias and
// all, which is exactly what a calibration run needs -- so publish it there
// even though no chip is present.
#if !defined(USE_FAKE_MAG) || defined(USE_FAKE_WHEEL)
#define PUBLISH_MAG
#endif

rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t mag_publisher;
rcl_subscription_t twist_subscriber;
rcl_publisher_t battery_publisher;
#ifdef USE_SAFETY_STOP
rcl_publisher_t safety_stop_publisher;
std_msgs__msg__Bool safety_stop_msg;
bool safety_stopped = false;
#endif
rcl_publisher_t range_publisher;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__MagneticField mag_msg;
geometry_msgs__msg__Twist twist_msg;
sensor_msgs__msg__BatteryState battery_msg;
sensor_msgs__msg__Range range_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
float prev_voltage;

enum states 
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

#ifdef USE_FAKE_WHEEL
FakeIMUFromWheels fake_imu;
#endif
#ifdef USE_FAKE_LD19
FakeLD19 fake_ld19;
#endif

ENCODER motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV);
ENCODER motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV);
ENCODER motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, COUNTS_PER_REV3, MOTOR3_ENCODER_INV);
ENCODER motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, COUNTS_PER_REV4, MOTOR4_ENCODER_INV);

Motor motor1_controller(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2_controller(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
Motor motor3_controller(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Motor motor4_controller(PWM_FREQUENCY, PWM_BITS, MOTOR4_INV, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

PID motor1_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor2_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor3_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);
PID motor4_pid(PWM_MIN, PWM_MAX, K_P, K_I, K_D);

Kinematics kinematics(
    Kinematics::LINO_BASE, 
    MOTOR_MAX_RPM, 
    MAX_RPM_RATIO, 
    MOTOR_OPERATING_VOLTAGE, 
    MOTOR_POWER_MAX_VOLTAGE, 
    WHEEL_DIAMETER, 
    LR_WHEELS_DISTANCE
);

Odometry odometry;
IMU imu;
MAG mag;

#ifndef BAUDRATE
#define BAUDRATE 921600
#endif

void setup() 
{
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(BAUDRATE);
#ifdef ESP32
    Serial.setRxBufferSize(1024);
#endif

#ifdef BOARD_INIT // board specific setup, must include Wire.begin
    BOARD_INIT
#else
    Wire.begin();
#endif

    initWifis();
    initOta();
#ifdef USE_FAKE_WHEEL
    // A bare module has nothing on the I2C bus, so probing it would fail and
    // the fatal loops below would trap the board before it ever connects. The
    // simulated IMU and magnetometer are computed from the simulated wheels
    // anyway, and would overwrite whatever a real sensor returned -- so skip
    // the hardware entirely and just prepare the two messages.
    fake_imu.initMsgs(imu_msg, mag_msg);
#else
    bool imu_ok = imu.init();
    if (!imu_ok) // take IMU failure as fatal
    {
        Serial.println("IMU init failed");
        syslog(LOG_INFO, "%s IMU init failed %lu", __FUNCTION__, millis());
        while (1)
        {
            flashLED(3); // flash 3 times
            runWifis();
            runOta();
        }
    }
    bool mag_ok = mag.init();
    if (!mag_ok) // take mag failure as fatal
    {
        Serial.println("MAG init failed");
        syslog(LOG_INFO, "%s MAG init failed %lu", __FUNCTION__, millis());
        while (1)
        {
            flashLED(4); // flash 4 times
            runWifis();
            runOta();
        }
    }
#endif
    initBattery();
    initRange();
#if defined(USE_FAKE_SONAR) && defined(USE_FAKE_LD19)
    // initRange() only sets this up when a real sensor is compiled in
    range_msg.header.frame_id =
        micro_ros_string_utilities_set(range_msg.header.frame_id, "sonar_link");
#endif
    initLidar(); // after wifi connected
#ifdef USE_FAKE_LD19
#ifdef LIDAR_RXD
    fake_ld19.begin(LIDAR_RXD, LIDAR_BAUDRATE);
#else
    fake_ld19.begin();
#endif
#endif
    battery_msg = getBattery();
    prev_voltage = battery_msg.voltage;

#ifdef MICRO_ROS_TRANSPORT_ARDUINO_WIFI
    set_microros_net_transports(AGENT_IP, AGENT_PORT);
#else
    set_microros_serial_transports(Serial);
#endif

#ifdef BOARD_INIT_LATE // board specific setup
    BOARD_INIT_LATE
#endif
    syslog(LOG_INFO, "%s Ready %lu", __FUNCTION__, millis());
}

#ifdef USE_FAKE_LD19
// Simulated wall contact indicator.
//
// LED_PIN may be -1 on boards with no addressable status LED, or LED_BUILTIN,
// which is a non-macro identifier the preprocessor evaluates as 0 -- so the
// guard is "defined and >= 0" and LED_BUILTIN boards stay enabled.
//
// The flash is timed rather than delayed: this runs inside the 50 Hz control
// path, and a delay() here would stall the whole loop.
#if defined(LED_PIN) && (LED_PIN) >= 0
#define FAKE_WALL_LED
#endif

static unsigned long fake_wall_led_off_at = 0;

static inline void fakeWallLedOn()
{
#ifdef FAKE_WALL_LED
    digitalWrite(LED_PIN, HIGH);
    fake_wall_led_off_at = millis() + 120;
#endif
}

static inline void fakeWallLedService()
{
#ifdef FAKE_WALL_LED
    if (fake_wall_led_off_at != 0 && (long)(millis() - fake_wall_led_off_at) >= 0)
    {
        digitalWrite(LED_PIN, LOW);
        fake_wall_led_off_at = 0;
    }
#endif
}
#endif

void loop() {
#ifdef USE_FAKE_LD19
    fakeWallLedService();
#endif
    switch (state) 
    {
        case WAITING_AGENT:
            EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
            break;
        case AGENT_AVAILABLE:
            syslog(LOG_INFO, "%s agent available %lu", __FUNCTION__, millis());
            state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (state == WAITING_AGENT) 
            {
                destroyEntities();
            }
            break;
        case AGENT_CONNECTED:
#ifndef USE_STAY_CONNECTED // Stay connected. Do not ping.
            EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
#endif
            if (state == AGENT_CONNECTED) 
            {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
            }
            break;
        case AGENT_DISCONNECTED:
            syslog(LOG_INFO, "%s agent disconnected %lu", __FUNCTION__, millis());
            fullStop();
            destroyEntities();
            state = WAITING_AGENT;
            break;
        default:
            break;
    }
    runWifis();
    runOta();
#ifdef WDT_TIMEOUT
    esp_task_wdt_reset();
#endif
#ifdef BOARD_LOOP // board specific loop
    BOARD_LOOP
#endif
#ifdef USE_FAKE_LD19
    fake_ld19.step();
#endif
}

void controlCallback(rcl_timer_t * timer, int64_t last_call_time) 
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) 
    {
       moveBase();
       publishData();
    }
}

void twistCallback(const void * msgin) 
{
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));

    prev_cmd_time = millis();
}

bool createEntities()
{
    syslog(LOG_INFO, "%s %lu", __FUNCTION__, millis());
    allocator = rcl_get_default_allocator();
    //create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    // create node
    RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));
    // create odometry publisher
    RCCHECK(rclc_publisher_init_default( 
        &odom_publisher, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        TOPIC_PREFIX "odom/unfiltered"
    ));
    // create IMU publisher
    RCCHECK(rclc_publisher_init_default( 
        &imu_publisher, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    // if we have magnetomter, use imu/data_raw for madgwick filter
#ifdef PUBLISH_MAG
        TOPIC_PREFIX "imu/data_raw"
#else
        TOPIC_PREFIX "imu/data"
#endif
    ));
#ifdef PUBLISH_MAG
    RCCHECK(rclc_publisher_init_default(
        &mag_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
        TOPIC_PREFIX "imu/mag"
    ));
#endif
#if defined(BATTERY_PIN) || defined(USE_INA219)
    // create battery pyblisher
    RCCHECK(rclc_publisher_init_default(
    &battery_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
    TOPIC_PREFIX "battery"
    ));
#endif
#ifdef USE_SAFETY_STOP
    // Tells ROS the robot stopped itself. The stop is a firmware reflex -- it
    // has to keep working when the ROS side is busy, wedged or disconnected --
    // so this publisher only reports the state, it never decides it.
    RCCHECK(rclc_publisher_init_default(
    &safety_stop_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    TOPIC_PREFIX "safety_stop"
    ));
#endif
#if defined(ECHO_PIN) || (defined(USE_FAKE_SONAR) && defined(USE_FAKE_LD19))
    // create range pyblisher
    RCCHECK(rclc_publisher_init_default(
    &range_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    TOPIC_PREFIX "sonar"
    ));
#endif
    // create twist command subscriber
    RCCHECK(rclc_subscription_init_default( 
        &twist_subscriber, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        TOPIC_PREFIX "cmd_vel"
    ));
    // create timer for actuating the motors at 50 Hz (1000/20)
    const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default2( 
        &control_timer, 
        &support,
        RCL_MS_TO_NS(control_timeout),
        (rcl_timer_callback_t) controlCallback,
        true
    ));
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, & allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, 
        &twist_subscriber, 
        &twist_msg, 
        &twistCallback, 
        ON_NEW_DATA
    ));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    // synchronize time with the agent
    syncTime();
    digitalWrite(LED_PIN, HIGH);

#ifdef USE_FAKE_WHEEL
    // A simulated robot has no way to be picked up and put back at the start,
    // and its pose is board state: it survives the host container, the agent
    // and the whole ROS stack being torn down and rebuilt. So a second test run
    // silently begins wherever the first one parked the robot -- and once that
    // is against a simulated wall, USE_SAFETY_STOP zeroes forward velocity and
    // navigation fails as "goal outside map" or "failed to make progress",
    // neither of which points at inherited state. A new agent session means a
    // new run, so start it from the origin.
    //
    // Real robots deliberately do not do this: odometry must stay continuous
    // across a reconnect, or the transform tree jumps under whatever is
    // localising against it.
    odometry.reset();
#ifdef USE_FAKE_LD19
    fake_ld19.updatePose(0.0f, 0.0f, 0.0f);
#endif
    syslog(LOG_INFO, "%s simulated pose reset to origin %lu", __FUNCTION__, millis());
#endif

    return true;
}

bool destroyEntities()
{
    syslog(LOG_INFO, "%s %lu", __FUNCTION__, millis());
    rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
    (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    RCSOFTCHECK(rcl_publisher_fini(&odom_publisher, &node));
    RCSOFTCHECK(rcl_publisher_fini(&imu_publisher, &node));
#ifdef PUBLISH_MAG
    RCSOFTCHECK(rcl_publisher_fini(&mag_publisher, &node));
#endif
#if defined(BATTERY_PIN) || defined(USE_INA219)
    RCSOFTCHECK(rcl_publisher_fini(&battery_publisher, &node));
#endif
#ifdef USE_SAFETY_STOP
    RCSOFTCHECK(rcl_publisher_fini(&safety_stop_publisher, &node));
#endif
#if defined(ECHO_PIN) || (defined(USE_FAKE_SONAR) && defined(USE_FAKE_LD19))
    RCSOFTCHECK(rcl_publisher_fini(&range_publisher, &node));
#endif
    RCSOFTCHECK(rcl_subscription_fini(&twist_subscriber, &node));
    RCSOFTCHECK(rcl_timer_fini(&control_timer));
    RCSOFTCHECK(rclc_executor_fini(&executor));
    RCSOFTCHECK(rcl_node_fini(&node))
    RCSOFTCHECK(rclc_support_fini(&support));

    digitalWrite(LED_PIN, HIGH);
    
    return true;
}

void fullStop()
{
    twist_msg.linear.x = 0.0;
    twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;

    motor1_controller.brake();
    motor2_controller.brake();
    motor3_controller.brake();
    motor4_controller.brake();
}

#ifdef USE_SAFETY_STOP
#ifndef SAFETY_STOP_RANGE
#define SAFETY_STOP_RANGE 0.25f     // metres ahead before forward motion is cut
#endif

// Forward range from whichever sensor is compiled in, or -1 when there is none
// to consult -- in which case nothing is blocked, because a missing sensor must
// not brake the robot.
static inline float rangeAheadOrNegative()
{
#if defined(USE_FAKE_SONAR) && defined(USE_FAKE_LD19)
    return fake_ld19.rangeAheadM();
#elif defined(ECHO_PIN)
    const float r = getRange().range;
    return isfinite(r) ? r : -1.0f;
#else
    return -1.0f;
#endif
}
#endif

void moveBase()
{
    // brake if there's no command received, or when it's only the first command sent
    if(((millis() - prev_cmd_time) >= 200)) 
    {
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.angular.z = 0.0;

        digitalWrite(LED_PIN, HIGH);
    }

#ifdef USE_SAFETY_STOP
    // Forward hazard stop, decided here rather than in ROS. A stop that has to
    // travel out on a topic, be reasoned about, and come back as cmd_vel is one
    // network round trip too slow, and does nothing at all if the ROS side is
    // wedged or the link drops. This runs every control cycle regardless.
    //
    // Only forward motion is blocked: reverse and rotation stay available, or
    // the robot would be stuck against the obstacle with no way to back off.
    {
        const float range = rangeAheadOrNegative();
        const bool blocked = (range >= 0.0f) && (range < (float)SAFETY_STOP_RANGE);
        if (blocked && twist_msg.linear.x > 0.0)
        {
            twist_msg.linear.x = 0.0;
            twist_msg.linear.y = 0.0;
        }
        if (blocked != safety_stopped)
        {
            safety_stopped = blocked;
            syslog(LOG_INFO, "%s safety stop %s at %.2f m %lu", __FUNCTION__,
                   blocked ? "engaged" : "cleared", range, millis());
        }
    }
#endif

    // get the required rpm for each motor based on required velocities, and base used
    Kinematics::rpm req_rpm = kinematics.getRPM(
        twist_msg.linear.x, 
        twist_msg.linear.y, 
        twist_msg.angular.z
    );

    // get the current speed of each motor
    float current_rpm1 = motor1_encoder.getRPM();
    float current_rpm2 = motor2_encoder.getRPM();
    float current_rpm3 = motor3_encoder.getRPM();
    float current_rpm4 = motor4_encoder.getRPM();

    // the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
    // the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
    int pwm1 = motor1_pid.compute(req_rpm.motor1, current_rpm1);
    int pwm2 = motor2_pid.compute(req_rpm.motor2, current_rpm2);
    int pwm3 = motor3_pid.compute(req_rpm.motor3, current_rpm3);
    int pwm4 = motor4_pid.compute(req_rpm.motor4, current_rpm4);
    motor1_controller.spin(pwm1);
    motor2_controller.spin(pwm2);
    motor3_controller.spin(pwm3);
    motor4_controller.spin(pwm4);
#ifdef USE_FAKE_WHEEL
    // close the loop in software: the simulated wheels follow the commanded PWM
    motor1_encoder.feed(pwm1);
    motor2_encoder.feed(pwm2);
    motor3_encoder.feed(pwm3);
    motor4_encoder.feed(pwm4);
#endif

    Kinematics::velocities current_vel = kinematics.getVelocities(
        current_rpm1, 
        current_rpm2, 
        current_rpm3, 
        current_rpm4
    );

    unsigned long now = millis();
    float vel_dt = (now - prev_odom_update) / 1000.0;
    prev_odom_update = now;
    odometry.update(
        vel_dt, 
        current_vel.linear_x, 
        current_vel.linear_y, 
        current_vel.angular_z
    );
#ifdef USE_FAKE_LD19
    // Stop the simulated robot at the simulated walls, and correct the
    // odometry to match, so /odom and /scan never disagree about where it is.
    float fake_x = odometry.getX();
    float fake_y = odometry.getY();
    bool hit_wall = fake_ld19.clampToRoom(fake_x, fake_y);
    if (hit_wall)
        odometry.setPosition(fake_x, fake_y);
    fake_ld19.updatePose(fake_x, fake_y, odometry.getHeading());
#else
    const bool hit_wall = false;
#endif
#ifdef USE_FAKE_WHEEL
    // The IMU rides on how the body actually moved, which is not what the
    // wheels claim once the robot is against a wall. Real hardware behaves the
    // same way: the wheels slip and keep reporting speed, while the IMU feels
    // no acceleration and the robot goes nowhere. Keeping that disagreement is
    // the only feedback there is that something was hit -- there is no bump
    // sensor, and odometry velocity alone never reveals it. Rotation survives,
    // since a robot pinned against a wall can still turn on the spot.
    fake_imu.update(
        hit_wall ? 0.0f : current_vel.linear_x,
        hit_wall ? 0.0f : current_vel.linear_y,
        current_vel.angular_z,
        vel_dt
    );
    fake_imu.setHeading(odometry.getHeading());
#endif
#ifdef USE_FAKE_LD19
    // Announce the contact once, on the way in. Driving into a wall holds the
    // clamp active for as long as the command lasts, so logging every 20 ms
    // cycle would bury the syslog in identical lines.
    static bool was_clamped = false;
    if (hit_wall && !was_clamped)
    {
        syslog(LOG_INFO, "%s fake wall contact at x %.2f y %.2f %lu",
               __FUNCTION__, fake_x, fake_y, millis());
        fakeWallLedOn();
    }
    was_clamped = hit_wall;
#endif
}

void publishData()
{
    static unsigned skip_dip = 0;
    odom_msg = odometry.getData();
#ifdef USE_FAKE_WHEEL
    // Every field these would return is overwritten just below, and on a bare
    // module the reads are two failing I2C transactions per publish, each one
    // stalling the loop for the bus timeout. Skip them.
    fake_imu.apply(imu_msg);
    // Simulated wheels mean a simulated heading, so the magnetometer has to
    // follow it: a real one left in the loop here would fight the fused yaw.
    fake_imu.applyMag(mag_msg);
#else
    imu_msg = imu.getData();
#ifdef USE_FAKE_IMU
    imu_msg.angular_velocity.z = odom_msg.twist.twist.angular.z;
#endif
    mag_msg = mag.getData();
#endif
#ifdef MAG_BIAS
    const float mag_bias[3] = MAG_BIAS;
    mag_msg.magnetic_field.x -= mag_bias[0];
    mag_msg.magnetic_field.y -= mag_bias[1];
    mag_msg.magnetic_field.z -= mag_bias[2];
#endif

    struct timespec time_stamp = getTime();

    odom_msg.header.stamp.sec = time_stamp.tv_sec;
    odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;

    imu_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;

#ifdef PUBLISH_MAG
    mag_msg.header.stamp.sec = time_stamp.tv_sec;
    mag_msg.header.stamp.nanosec = time_stamp.tv_nsec;
#endif

    RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
#ifdef PUBLISH_MAG
    RCSOFTCHECK(rcl_publish(&mag_publisher, &mag_msg, NULL));
#endif
    RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
#if defined(BATTERY_PIN) || defined(USE_INA219)
    battery_msg = getBattery();
    battery_msg.header.stamp.sec = time_stamp.tv_sec;
    battery_msg.header.stamp.nanosec = time_stamp.tv_nsec;
#ifdef BATTERY_DIP
    if (!skip_dip && battery_msg.voltage > 1.0  && battery_msg.voltage < prev_voltage * BATTERY_DIP) {
        RCSOFTCHECK(rcl_publish(&battery_publisher, &battery_msg, NULL));
    syslog(LOG_WARNING, "%s voltage dip %.2f", __FUNCTION__, battery_msg.voltage);
        skip_dip = 5;
    }
    if (skip_dip) skip_dip--;
#endif
    battery_msg.voltage = prev_voltage = battery_msg.voltage * 0.01 + prev_voltage * 0.99;
    EXECUTE_EVERY_N_MS(BATTERY_TIMER, {
        getBatteryPercentage(&battery_msg);
        RCSOFTCHECK(rcl_publish(&battery_publisher, &battery_msg, NULL)) });
#endif
#ifdef USE_SAFETY_STOP
    safety_stop_msg.data = safety_stopped;
    RCSOFTCHECK(rcl_publish(&safety_stop_publisher, &safety_stop_msg, NULL));
#endif
#if defined(USE_FAKE_SONAR) && defined(USE_FAKE_LD19)
    // A simulated ultrasonic sensor, raycast from the same room the simulated
    // LiDAR uses. This is the robot's own feedback that something is ahead --
    // wheel odometry cannot provide it, because the wheels keep turning when
    // the robot is stopped against something.
    EXECUTE_EVERY_N_MS(RANGE_TIMER, {
        range_msg.range = fake_ld19.rangeAheadM();
        range_msg.field_of_view = (float)FAKE_SONAR_CONE_DEG * (float)DEG_TO_RAD;
        range_msg.min_range = 0.02;
        range_msg.max_range = 4.0;
        range_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
        range_msg.header.stamp.sec = time_stamp.tv_sec;
        range_msg.header.stamp.nanosec = time_stamp.tv_nsec;
        RCSOFTCHECK(rcl_publish(&range_publisher, &range_msg, NULL)) });
#elif defined(ECHO_PIN)
    EXECUTE_EVERY_N_MS(RANGE_TIMER, {
        range_msg = getRange();
        range_msg.header.stamp.sec = time_stamp.tv_sec;
        range_msg.header.stamp.nanosec = time_stamp.tv_nsec;
        RCSOFTCHECK(rcl_publish(&range_publisher, &range_msg, NULL)) });
#endif
}

bool syncTime()
{
    const int timeout_ms = 1000;
    if (rmw_uros_epoch_synchronized()) return true; // synchronized previously
    // get the current time from the agent
    RCCHECK(rmw_uros_sync_session(timeout_ms));
    if (rmw_uros_epoch_synchronized()) {
#if (_POSIX_TIMERS > 0)
        // Get time in milliseconds or nanoseconds
        int64_t time_ns = rmw_uros_epoch_nanos();
    timespec tp;
    tp.tv_sec = time_ns / 1000000000;
    tp.tv_nsec = time_ns % 1000000000;
    clock_settime(CLOCK_REALTIME, &tp);
#else
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - millis();
#endif
    return true;
    }
    return false;
}

struct timespec getTime()
{
    struct timespec tp = {0};
#if (_POSIX_TIMERS > 0)
    clock_gettime(CLOCK_REALTIME, &tp);
#else
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
#endif
    return tp;
}

void rclErrorLoop() 
{
    while(true)
    {
        flashLED(2); // flash 2 times
        runOta();
    }
}

void flashLED(int n_times)
{
    for(int i=0; i<n_times; i++)
    {
        digitalWrite(LED_PIN, HIGH);
        delay(150);
        digitalWrite(LED_PIN, LOW);
        delay(150);
    }
    delay(1000);
}
