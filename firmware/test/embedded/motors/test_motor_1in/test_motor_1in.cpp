/// \file test_motor_1in.cpp
/// \brief Tests for Motor drivers with 1 Direction Pin(INA) and 1 PWM(ENABLE) pin.

#include <Arduino.h>
#include <unity.h>

#define USE_GENERIC_1_IN_MOTOR_DRIVER 
#include "motor.h"


constexpr int g_pwm_bits = 10;  // PWM Resolution of the microcontroller
constexpr int g_pwm_frequency = 20000;  // PWM Frequency
constexpr int g_motor_pwm_pin = 5;
constexpr int g_motor_direction_pin_a = 6;

// TODO figure out whether setUp() and tearDown() are needed
// TODO figure out how to read PWM
// TODO figure out how to meaningfully test brake

/** 
 * \brief Test the forward pin of the generic motor.
 */
void test_generic1_forward_pin() {
    Motor motor(g_pwm_frequency, g_pwm_bits, false, 
                g_motor_pwm_pin, g_motor_direction_pin_a);
    motor.spin(100);
    TEST_ASSERT_EQUAL(HIGH, digitalRead(g_motor_direction_pin_a));
}

/** 
 * \brief Test the forward pin of the generic motor with inverted configuration.
 */
void test_generic1_forward_pin_inv() {
    Motor motor(g_pwm_frequency, g_pwm_bits, true, 
                g_motor_pwm_pin, g_motor_direction_pin_a);
    motor.spin(100);
    TEST_ASSERT_EQUAL(LOW, digitalRead(g_motor_direction_pin_a));
}

/** 
 * \brief Test the reverse pin of the generic motor.
 */
void test_generic1_reverse_pin() {
    Motor motor(g_pwm_frequency, g_pwm_bits, false, 
                g_motor_pwm_pin, g_motor_direction_pin_a);
    motor.spin(-100);
    TEST_ASSERT_EQUAL(LOW, digitalRead(g_motor_direction_pin_a));
}

/** 
 * \brief Test the reverse pin of the generic motor with inverted configuration.
 */
void test_generic1_reverse_pin_inv() {
    Motor motor(g_pwm_frequency, g_pwm_bits, true, 
                g_motor_pwm_pin, g_motor_direction_pin_a);
    motor.spin(-100);
    TEST_ASSERT_EQUAL(HIGH, digitalRead(g_motor_direction_pin_a));
}

/** 
 * \brief Main function to run all the motor tests.
 * \param argc Argument count.
 * \param argv Argument values.
 * \return Test results.
 */
int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_generic1_forward_pin);
    RUN_TEST(test_generic1_forward_pin_inv);
    RUN_TEST(test_generic1_reverse_pin);
    RUN_TEST(test_generic1_reverse_pin_inv);
    UNITY_END();
}
