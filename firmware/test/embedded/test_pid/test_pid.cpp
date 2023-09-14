/// \file test_pid.cpp
/// \brief Unit tests for the PID controller class.
///
/// This file contains unit tests for the PID controller class to ensure its correct functionality.


#include <unity.h>
#include "pid.h"


constexpr int g_pwm_bits = 10;
constexpr double g_pwm_max = pow(2, g_pwm_bits) - 1;
constexpr double g_pwm_min = -g_pwm_max;
constexpr double g_k_p = 0.6;  // P constant
constexpr double g_k_i = 0.8;  // I constant
constexpr double g_k_d = 0.5;  // D constant

// TODO swap to using TEST_ASSERT_GREATER_THAN_DOUBLE and TEST_ASSERT_FLOAT_WITHIN once unity is 
// released in new version updated on PlatformIO registry. Workarounds until then.
// aware that can manually include newer version as libdep, but causes problems

constexpr double fp_tolerance = 1e-10;

PID g_dut_pid(g_pwm_min, g_pwm_max, g_k_p, g_k_i, g_k_d);

void setUp(void) {
    // Reset PID internal states by reinitializing
    g_dut_pid = PID(g_pwm_min, g_pwm_max, g_k_p, g_k_i, g_k_d);
}

/** 
 * \brief Test that a system in the desired state should not have any correction.
 */
void test_compute_zero_error() {
    double output = g_dut_pid.compute(100, 100);

    // Check if output is within a small tolerance of 0
    // HACK for TEST_ASSERT_FLOAT_WITHIN(1e-10, 0, output); 
    bool pid_near_zero = false;
    if (abs(output) < fp_tolerance) {
        pid_near_zero = true;
    }

    TEST_ASSERT_TRUE(pid_near_zero);
}

/** 
 * \brief Test if feedback is positive when measured value is less than the setpoint.
 */
void test_positive_feedback() {
    double output = g_dut_pid.compute(100, 90);

    // HACK for TEST_ASSERT_GREATER_THAN_DOUBLE(0, output);
    bool pid_greater_than_zero = output > 0;
    TEST_ASSERT_TRUE(pid_greater_than_zero);
}

/** 
 * \brief Test if feedback is negative when measured value is greater than the setpoint.
 */
void test_negative_feedback() {
    double output = g_dut_pid.compute(90, 100);

    // HACK for TEST_ASSERT_LESS_THAN_DOUBLE(0, output);
    bool pid_less_than_zero = output < 0;
    TEST_ASSERT_TRUE(pid_less_than_zero);
}

/** 
 * \brief Test that the PID output is constrained between the minimum and maximum values.
 */
void test_output_constrain() {
    // This will produce a large error
    double output = g_dut_pid.compute(1000, 0); 
    TEST_ASSERT_TRUE(output <= g_pwm_max && output >= g_pwm_min);
}

// TODO test updating constants by setting KI and KD 0
// TODO write a test for integral windup
// TODO model a real PID system

/** 
 * \brief Main function to run all the PID tests.
 * \param argc Argument count.
 * \param argv Argument values.
 * \return Test results.
 */
int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_compute_zero_error);
    RUN_TEST(test_positive_feedback);
    RUN_TEST(test_negative_feedback);
    RUN_TEST(test_output_constrain);
    UNITY_END();
}
