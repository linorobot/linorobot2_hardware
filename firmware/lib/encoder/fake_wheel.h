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

#ifndef FAKE_WHEEL_H
#define FAKE_WHEEL_H

#include <Arduino.h>

// Simulated drivetrain for a bare module with no motors or encoders attached.
//
// Each wheel is modelled as a DC motor driving a share of the robot's mass:
//
//   duty      = pwm / PWM_MAX                 commanded power
//   no_load   = duty * MOTOR_MAX_RPM          speed the motor would settle at
//   torque   ~= no_load - rpm                 back-EMF: torque falls as it spins up
//   d(rpm)/dt = torque / tau_eff              tau_eff scales with robot mass
//
// so the wheel accelerates hard when the error is large, tails off as it
// approaches speed, coasts down against friction when power is cut, and takes
// longer to do all of it on a heavier robot. Acceleration is clamped to a
// traction/current limit, and a stall band keeps a weak duty from creeping.
// The reported RPM carries white noise, so the PID has something real to
// correct against and the odometry drifts the way it does on real hardware.

// The robot's total weight is optional robot data from the config engine, which
// emits it as ROBOT_WEIGHT. Use it when it is there, so the simulated robot has
// the inertia of the one that was configured.
#ifndef FAKE_ROBOT_MASS
    #ifdef ROBOT_WEIGHT
        #define FAKE_ROBOT_MASS ROBOT_WEIGHT
    #else
        #define FAKE_ROBOT_MASS 3.5     // simulated robot mass (kg)
    #endif
#endif

#ifndef FAKE_WHEEL_TAU_MS
#define FAKE_WHEEL_TAU_MS 150       // spin-up time constant (ms) at FAKE_WHEEL_REF_MASS
#endif

#ifndef FAKE_WHEEL_REF_MASS
#define FAKE_WHEEL_REF_MASS 3.5     // mass FAKE_WHEEL_TAU_MS was measured at (kg)
#endif

#ifndef FAKE_WHEEL_MAX_ACCEL_RPM
#define FAKE_WHEEL_MAX_ACCEL_RPM 900.0  // traction/current limit (RPM per second)
#endif

#ifndef FAKE_WHEEL_FRICTION
#define FAKE_WHEEL_FRICTION 0.06    // viscous drag, fraction of current RPM per second
#endif

#ifndef FAKE_WHEEL_STALL_DUTY
#define FAKE_WHEEL_STALL_DUTY 0.04  // duty below which the motor cannot break static friction
#endif

#ifndef FAKE_WHEEL_NOISE_RPM
#define FAKE_WHEEL_NOISE_RPM 1.0    // +/- peak white noise on the reported RPM
#endif

// +/- peak white noise, scaled by amplitude
static inline float fakeWheelNoise(float amplitude)
{
    return ((float)random(-1000, 1001) / 1000.0) * amplitude;
}

class FakeEncoder
{
private:
    int counts_per_rev_ = -1;
    bool invert_ = false;
    float duty_ = 0.0;              // commanded duty cycle, -1.0 .. 1.0
    float wheel_rpm_ = 0.0;         // simulated wheel speed
    double ticks_ = 0.0;            // simulated tick accumulator
    unsigned long prev_update_time_ = 0;

    // advance the wheel model to now
    void integrate()
    {
        unsigned long current_time = micros();
        unsigned long dt = current_time - prev_update_time_;
        prev_update_time_ = current_time;
        // first call, or a micros() rollover: seed the clock, don't step the model
        if (dt == 0 || dt > 1000000UL) return;
        float dts = (float)dt / 1000000.0;

        // heavier robot, more inertia per wheel, slower response
        float tau = (FAKE_WHEEL_TAU_MS / 1000.0) *
                    ((float)FAKE_ROBOT_MASS / (float)FAKE_WHEEL_REF_MASS);
        if (tau < 0.001) tau = 0.001;

        float no_load_rpm = duty_ * (float)MOTOR_MAX_RPM;
        // below the stall band the motor cannot hold the wheel against friction
        if (fabsf(duty_) < (float)FAKE_WHEEL_STALL_DUTY) no_load_rpm = 0.0;

        // back-EMF: driving torque is proportional to the remaining speed error
        float accel = (no_load_rpm - wheel_rpm_) / tau;
        // viscous friction always opposes motion
        accel -= wheel_rpm_ * (float)FAKE_WHEEL_FRICTION;
        // traction and current limit the achievable acceleration
        if (accel > (float)FAKE_WHEEL_MAX_ACCEL_RPM) accel = (float)FAKE_WHEEL_MAX_ACCEL_RPM;
        if (accel < -(float)FAKE_WHEEL_MAX_ACCEL_RPM) accel = -(float)FAKE_WHEEL_MAX_ACCEL_RPM;

        wheel_rpm_ += accel * dts;
        // an unpowered wheel settles rather than creeping forever
        if (no_load_rpm == 0.0 && fabsf(wheel_rpm_) < 0.5) wheel_rpm_ = 0.0;

        // accumulate ticks so read() stays consistent with getRPM()
        ticks_ += (double)wheel_rpm_ / 60.0 * counts_per_rev_ * dts;
    }

public:
    FakeEncoder(int pin1, int pin2, int counts_per_rev, bool invert = false)
    {
        // The pins are deliberately ignored. Fake wheel mode exists for boards
        // with nothing wired, where the encoder pins are normally left unset
        // (-1); keying off them would leave every simulated wheel at 0 RPM,
        // which is the one case this class is for. Nothing here touches GPIO.
        // Which wheels actually count is the kinematics' decision, not the pin
        // map: Kinematics::getVelocities() zeroes motors 3 and 4 on a
        // differential base regardless of what the encoders report.
        (void)pin1;
        (void)pin2;
        counts_per_rev_ = (counts_per_rev > 0) ? counts_per_rev : 1;
        invert_ = invert;
    }

    // called by the control loop with the PWM just handed to the motor driver
    void feed(int pwm)
    {
        if (counts_per_rev_ < 0) return;
        integrate();
        if (invert_) pwm *= -1;
        // PWM_MAX expands to an unparenthesized expression (`pow(2, PWM_BITS) - 1`),
        // so it has to be wrapped before dividing or the `- 1` escapes the cast and
        // lands outside the division, turning a small duty into nearly full reverse.
        float duty = (float)pwm / (float)(PWM_MAX);
        if (duty > 1.0) duty = 1.0;
        if (duty < -1.0) duty = -1.0;
        duty_ = duty;
    }

    float getRPM()
    {
        if (counts_per_rev_ < 0) return 0.0;
        integrate();
        return wheel_rpm_ + fakeWheelNoise((float)FAKE_WHEEL_NOISE_RPM);
    }

    inline int32_t read()
    {
        if (counts_per_rev_ < 0) return 0;
        integrate();
        return (int32_t)ticks_;
    }

    inline void write(int32_t p)
    {
        if (counts_per_rev_ < 0) return;
        ticks_ = (double)p;
    }
};

#ifndef FAKE_IMU_GRAVITY
#define FAKE_IMU_GRAVITY 9.81       // specific force reported on Z when level
#endif

#ifndef FAKE_IMU_ACCEL_TAU_MS
#define FAKE_IMU_ACCEL_TAU_MS 60    // accelerometer band limit (ms)
#endif

#ifndef FAKE_IMU_ACCEL_NOISE
#define FAKE_IMU_ACCEL_NOISE 0.05   // +/- peak accelerometer noise (m/s^2)
#endif

// A real MEMS IMU is not a clean derivative of the truth: it has a fixed bias,
// a bias that wanders slowly with temperature, a scale-factor error, and white
// noise on top. Fusion (madgwick, the EKF) exists to fight exactly that, so a
// perfect simulated IMU would make the whole estimation stack look better than
// it is on hardware.
#ifndef FAKE_IMU_GYRO_BIAS
#define FAKE_IMU_GYRO_BIAS 0.004f       // fixed gyro bias (rad/s)
#endif

#ifndef FAKE_IMU_GYRO_DRIFT
#define FAKE_IMU_GYRO_DRIFT 0.0015f     // gyro bias random walk (rad/s per sqrt(s))
#endif

#ifndef FAKE_IMU_ACCEL_BIAS
#define FAKE_IMU_ACCEL_BIAS 0.03f       // fixed accelerometer bias (m/s^2)
#endif

#ifndef FAKE_IMU_ACCEL_DRIFT
#define FAKE_IMU_ACCEL_DRIFT 0.01f      // accelerometer bias random walk (m/s^2 per sqrt(s))
#endif

#ifndef FAKE_IMU_SCALE_ERROR
#define FAKE_IMU_SCALE_ERROR 0.01f      // scale-factor error, fraction of reading
#endif

#ifndef FAKE_MAG_FIELD_T
#define FAKE_MAG_FIELD_T 50e-6f     // Simulated field strength (Tesla, ~Earth)
#endif

#ifndef FAKE_MAG_ROOM_HEADING
#define FAKE_MAG_ROOM_HEADING 0.0f  // Heading (rad) of the room's +X axis vs the field
#endif

#ifndef FAKE_MAG_NOISE_T
#define FAKE_MAG_NOISE_T 0.5e-6f    // +/- peak magnetometer noise (Tesla)
#endif

// Hard-iron offset. A magnetometer mounted on a robot always sits next to
// motors, batteries and steel, which add a fixed vector to every reading and
// pull the heading round with the robot. Simulating it means the magnetometer
// calibration routine has a real offset to discover and remove, instead of
// converging on zero and proving nothing.
#ifndef FAKE_MAG_BIAS_X
#define FAKE_MAG_BIAS_X 6.0e-6f
#endif
#ifndef FAKE_MAG_BIAS_Y
#define FAKE_MAG_BIAS_Y -4.0e-6f
#endif
#ifndef FAKE_MAG_BIAS_Z
#define FAKE_MAG_BIAS_Z 2.5e-6f
#endif

#ifndef FAKE_IMU_GYRO_NOISE
#define FAKE_IMU_GYRO_NOISE 0.005   // +/- peak gyroscope noise (rad/s)
#endif

// Derives IMU readings from the simulated body motion, so the accelerometer
// and gyroscope agree with the wheel encoders instead of reading zero.
// Hold a wandering bias inside a plausible envelope.
static inline float clampBias(float v, float limit)
{
    if (limit < 0.0f) limit = -limit;
    if (v >  limit) return  limit;
    if (v < -limit) return -limit;
    return v;
}

class FakeIMUFromWheels
{
private:
    float linear_x_ = 0.0;          // latest body velocities
    float linear_y_ = 0.0;
    float angular_z_ = 0.0;
    float accel_x_ = 0.0;           // smoothed body accelerations
    float accel_y_ = 0.0;
    float heading_ = 0.0;           // simulated yaw, for the magnetometer
    // slowly wandering sensor biases, seeded to a fixed offset and then walked
    // Start calibrated. A real IMU has its static bias measured and subtracted
    // at startup -- IMUInterface::init() calls calibrateGyro(), which averages
    // 40 samples and stores the offset. Fake wheel mode never calls that (there
    // is no chip to talk to), so seeding these with the full bias simulated an
    // IMU that had skipped its own calibration: the gyro read a steady offset
    // forever, the EKF integrated it, and yaw walked away from the wheels.
    float gyro_bias_z_ = 0.0f;
    float accel_bias_x_ = 0.0f;
    float accel_bias_y_ = 0.0f;

public:
    // With no real IMU on the bus, imu.getData() / mag.getData() are never
    // called, and the two messages never get the frame and covariances those
    // calls would have filled in. Set them once here instead: without a
    // frame_id the messages are dropped by tf, and with zero covariance the
    // EKF treats the simulated sensor as exact.
    void initMsgs(sensor_msgs__msg__Imu &imu_msg,
                  sensor_msgs__msg__MagneticField &mag_msg)
    {
        const float accel_cov[3] = ACCEL_COV;
        const float gyro_cov[3] = GYRO_COV;
        const float ori_cov[3] = ORI_COV;
        const float mag_cov[3] = MAG_COV;

        imu_msg.header.frame_id =
            micro_ros_string_utilities_set(imu_msg.header.frame_id, "imu_link");
        mag_msg.header.frame_id =
            micro_ros_string_utilities_set(mag_msg.header.frame_id, "imu_link");

        for (int i = 0; i < 3; i++)
        {
            const int d = i * 4;    // 0, 4, 8: the diagonal of a 3x3 row-major
            imu_msg.linear_acceleration_covariance[d] = accel_cov[i];
            imu_msg.angular_velocity_covariance[d] = gyro_cov[i];
            imu_msg.orientation_covariance[d] = ori_cov[i];
            mag_msg.magnetic_field_covariance[d] = mag_cov[i];
        }
    }

    // fed from the body velocities the kinematics derived from the wheels
    void update(float linear_x, float linear_y, float angular_z, float dt)
    {
        if (dt > 0.0)
        {
            // differencing velocity at the loop rate is spiky; a real
            // accelerometer is band limited, so ease into the new value
            float raw_x = (linear_x - linear_x_) / dt;
            float raw_y = (linear_y - linear_y_) / dt;
            float alpha = 1.0 - expf(-dt / (FAKE_IMU_ACCEL_TAU_MS / 1000.0));
            accel_x_ += (raw_x - accel_x_) * alpha;
            accel_y_ += (raw_y - accel_y_) * alpha;

            // Random walk: the step scales with sqrt(dt), so the drift rate is
            // independent of how often this happens to be called.
            //
            // Bounded, because a free random walk has no bound and this one had
            // none: the longer the board ran, the further the bias wandered,
            // and it does not come back. Measured after a long session the
            // stationary gyro read 0.01813 rad/s -- 4.5x the nominal bias --
            // and the EKF turned that into 0.87 deg/s of yaw drift, 52 deg in a
            // minute while the robot stood still. Real parts do not do that:
            // bias instability wanders within an envelope. Clamping to the
            // nominal bias keeps the drift a filter has to cope with, without
            // letting uptime decide whether SLAM works.
            const float rw = sqrtf(dt);
            gyro_bias_z_  += fakeWheelNoise((float)FAKE_IMU_GYRO_DRIFT) * rw;
            accel_bias_x_ += fakeWheelNoise((float)FAKE_IMU_ACCEL_DRIFT) * rw;
            accel_bias_y_ += fakeWheelNoise((float)FAKE_IMU_ACCEL_DRIFT) * rw;
            gyro_bias_z_  = clampBias(gyro_bias_z_,  (float)FAKE_IMU_GYRO_BIAS);
            accel_bias_x_ = clampBias(accel_bias_x_, (float)FAKE_IMU_ACCEL_BIAS);
            accel_bias_y_ = clampBias(accel_bias_y_, (float)FAKE_IMU_ACCEL_BIAS);
        }
        linear_x_ = linear_x;
        linear_y_ = linear_y;
        angular_z_ = angular_z;
    }

    // The simulated robot turns, so a magnetometer stuck at a constant vector
    // would disagree with the yaw the wheels report and drag any heading fusion
    // (madgwick, EKF) away from the truth. Rotate a fixed world field into the
    // body frame instead, so the mag agrees with the simulated room: the field
    // points along the room's +X axis, offset by FAKE_MAG_ROOM_HEADING.
    void setHeading(float heading) { heading_ = heading; }

    void applyMag(sensor_msgs__msg__MagneticField &mag_msg)
    {
        const float theta = heading_ - (float)FAKE_MAG_ROOM_HEADING;
        const float b = (float)FAKE_MAG_FIELD_T;
        // The world field points along +Y, i.e. North in the ENU frame ROS uses,
        // because that is the direction imu_filter_madgwick assumes when it
        // derives heading from the magnetometer. Pointing it along +X instead
        // is physically just as valid but leaves the fused yaw a fixed ~90 deg
        // from the wheel odometry's, and the EKF then has two heading sources
        // that disagree by a quarter turn: it splits the difference, drags the
        // pose sideways during a pure rotation, and the SLAM map comes out
        // sheared. Measured before this: /odom translated 3.9 m while the robot
        // only spun in place.
        //
        // Rotating the world vector (0, b) into the body frame by -theta:
        //   x =  b sin(theta)      y =  b cos(theta)
        // then spoiled by the hard-iron offset calibration is meant to find,
        // and white noise.
        mag_msg.magnetic_field.x =
            b * sinf(theta) + (float)FAKE_MAG_BIAS_X + fakeWheelNoise((float)FAKE_MAG_NOISE_T);
        mag_msg.magnetic_field.y =
            b * cosf(theta) + (float)FAKE_MAG_BIAS_Y + fakeWheelNoise((float)FAKE_MAG_NOISE_T);
        mag_msg.magnetic_field.z =
            (float)FAKE_MAG_BIAS_Z + fakeWheelNoise((float)FAKE_MAG_NOISE_T);
    }

    void apply(sensor_msgs__msg__Imu &imu_msg)
    {
        // a real accelerometer measures specific force: body accel, plus the
        // centripetal term while turning, plus gravity held up by the floor
        // true specific force, then spoiled the way a real sensor spoils it:
        // scale error on the signal, a wandering bias, then white noise
        const float k = 1.0f + (float)FAKE_IMU_SCALE_ERROR;
        const float ax = accel_x_ - angular_z_ * linear_y_;
        const float ay = accel_y_ + angular_z_ * linear_x_;

        imu_msg.linear_acceleration.x =
            ax * k + accel_bias_x_ + fakeWheelNoise((float)FAKE_IMU_ACCEL_NOISE);
        imu_msg.linear_acceleration.y =
            ay * k + accel_bias_y_ + fakeWheelNoise((float)FAKE_IMU_ACCEL_NOISE);
        imu_msg.linear_acceleration.z =
            (float)FAKE_IMU_GRAVITY + fakeWheelNoise((float)FAKE_IMU_ACCEL_NOISE);

        imu_msg.angular_velocity.x = fakeWheelNoise((float)FAKE_IMU_GYRO_NOISE);
        imu_msg.angular_velocity.y = fakeWheelNoise((float)FAKE_IMU_GYRO_NOISE);
        imu_msg.angular_velocity.z =
            angular_z_ * k + gyro_bias_z_ + fakeWheelNoise((float)FAKE_IMU_GYRO_NOISE);
    }
};

#ifdef USE_FAKE_WHEEL
    #define ENCODER FakeEncoder
#else
    #define ENCODER Encoder
#endif

#endif
