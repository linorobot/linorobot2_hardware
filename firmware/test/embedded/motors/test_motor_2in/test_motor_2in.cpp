/// \file test_motor_2in.cpp
/// \brief Tests for Motor drivers with 2 Direction Pins(INA, INB) and 1 PWM(ENABLE) pin.
/// Examples include L298, L293, VNH5019.

#include <Arduino.h>
#include <unity.h>

#define USE_GENERIC_2_IN_MOTOR_DRIVER 
#include "motor.h"


constexpr int g_pwm_bits = 10;  // PWM Resolution of the microcontroller
constexpr int g_pwm_frequency = 20000;  // PWM Frequency
constexpr int g_motor_pwm_pin = 5;
constexpr int g_motor_direction_pin_a = 6;
constexpr int g_motor_direction_pin_b = 8;

// TODO figure out whether setUp() and tearDown() are needed
// TODO figure out how to read PWM
// TODO figure out how to meaningfully test brake

/** 
 * \brief Test the forward pins of the generic motor.
 */
void test_generic2_forward_pins() {
    Motor motor(g_pwm_frequency, g_pwm_bits, false, 
                g_motor_pwm_pin, g_motor_direction_pin_a, g_motor_direction_pin_b);
    motor.spin(100);
    TEST_ASSERT_EQUAL(HIGH, digitalRead(g_motor_direction_pin_a));
    TEST_ASSERT_EQUAL(LOW, digitalRead(g_motor_direction_pin_b));
}

/** 
 * \brief Test the forward pins of the generic motor with inverted configuration.
 */
void test_generic2_forward_pins_inv() {
    Motor motor(g_pwm_frequency, g_pwm_bits, true, 
                g_motor_pwm_pin, g_motor_direction_pin_a, g_motor_direction_pin_b);
    motor.spin(100);
    TEST_ASSERT_EQUAL(LOW, digitalRead(g_motor_direction_pin_a));
    TEST_ASSERT_EQUAL(HIGH, digitalRead(g_motor_direction_pin_b));
}

/** 
 * \brief Test the reverse pins of the generic motor.
 */
void test_generic2_reverse_pins() {
    Motor motor(g_pwm_frequency, g_pwm_bits, false, 
                g_motor_pwm_pin, g_motor_direction_pin_a, g_motor_direction_pin_b);
    motor.spin(-100);
    TEST_ASSERT_EQUAL(LOW, digitalRead(g_motor_direction_pin_a));
    TEST_ASSERT_EQUAL(HIGH, digitalRead(g_motor_direction_pin_b));
}

/** 
 * \brief Test the reverse pins of the generic motor with inverted configuration.
 */
void test_generic2_reverse_pins_inv() {
    Motor motor(g_pwm_frequency, g_pwm_bits, true, 
                g_motor_pwm_pin, g_motor_direction_pin_a, g_motor_direction_pin_b);
    motor.spin(-100);
    TEST_ASSERT_EQUAL(HIGH, digitalRead(g_motor_direction_pin_a));
    TEST_ASSERT_EQUAL(LOW, digitalRead(g_motor_direction_pin_b));
}

/** 
 * \brief Main function to run all the motor tests.
 * \param argc Argument count.
 * \param argv Argument values.
 * \return Test results.
 */
int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_generic2_forward_pins);
    RUN_TEST(test_generic2_forward_pins_inv);
    RUN_TEST(test_generic2_reverse_pins);
    RUN_TEST(test_generic2_reverse_pins_inv);
    UNITY_END();
}
