#include <Arduino.h>
#include <micro_ros_utilities/string_utilities.h>
#include <sensor_msgs/msg/range.h>
#include "config.h"

// define sound speed in m/uS
#define SOUND_SPEED 0.00034
#define TIMEOUT_US 30000 // 30ms (~5.1m max range)
#define MIN_DURATION_US 115 // ~2cm min range
#define FOV (15 * 0.0174533) // field of view rad
#define MIN_RANGE 0.02 // 2cm
#define MAX_RANGE (SOUND_SPEED * TIMEOUT_US / 2.0 * 0.95)

#ifdef ECHO_PIN
enum SonarState {
    SONAR_IDLE,
    SONAR_TRIGGERED,
    SONAR_WAIT_ECHO_FALL
};

static volatile SonarState sonar_state = SONAR_IDLE;
static volatile uint32_t echo_start_us = 0;
static volatile uint32_t echo_duration_us = 0;
static volatile bool new_reading_available = false;
static uint32_t last_trigger_us = 0;

#if defined(ESP32) || defined(ESP8266)
void IRAM_ATTR echoPinISR()
#else
void echoPinISR()
#endif
{
    uint32_t now = micros();
    if (digitalRead(ECHO_PIN) == HIGH) {
        if (sonar_state == SONAR_TRIGGERED || sonar_state == SONAR_IDLE) {
            echo_start_us = now;
            sonar_state = SONAR_WAIT_ECHO_FALL;
        }
    } else {
        if (sonar_state == SONAR_WAIT_ECHO_FALL) {
            uint32_t duration = now - echo_start_us;
            if (duration >= MIN_DURATION_US && duration <= TIMEOUT_US) {
                echo_duration_us = duration;
                new_reading_available = true;
            } else {
                echo_duration_us = 0; // out of bounds
                new_reading_available = true;
            }
            sonar_state = SONAR_IDLE;
        }
    }
}
#endif

sensor_msgs__msg__Range range_msg_;

sensor_msgs__msg__Range getRange()
{
#ifdef TRIG_PIN
    uint32_t now = micros();

    // 1. Check for timeout if waiting for echo from far-away object
    if (sonar_state == SONAR_WAIT_ECHO_FALL || sonar_state == SONAR_TRIGGERED) {
        if ((now - echo_start_us) > TIMEOUT_US || (now - last_trigger_us) > TIMEOUT_US) {
            range_msg_.range = +INFINITY;
            sonar_state = SONAR_IDLE;
            new_reading_available = false;
        }
    }

    // 2. If a valid reading arrived from ISR, update range
    if (new_reading_available) {
        if (echo_duration_us > 0) {
            range_msg_.range = (float)(echo_duration_us * SOUND_SPEED / 2.0);
        } else {
            range_msg_.range = +INFINITY;
        }
        new_reading_available = false;
    }

    range_msg_.field_of_view = FOV;
    range_msg_.min_range = MIN_RANGE;
    range_msg_.max_range = MAX_RANGE;

    // 3. Trigger next measurement only if previous echo finished (non-blocking 10us)
    if (sonar_state == SONAR_IDLE && (now - last_trigger_us >= 25000)) {
        sonar_state = SONAR_TRIGGERED;
        last_trigger_us = now;
        echo_start_us = now;
        digitalWrite(TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN, LOW);
    }
#endif
    return range_msg_;
}

void initRange()
{
#ifdef TRIG_PIN // ultrasonic sensor HC-SR04
    range_msg_.header.frame_id = micro_ros_string_utilities_set(range_msg_.header.frame_id, "sonar_link");
    pinMode(TRIG_PIN, OUTPUT);
    digitalWrite(TRIG_PIN, LOW);
#endif
#ifdef ECHO_PIN
    pinMode(ECHO_PIN, INPUT);
    sonar_state = SONAR_IDLE;
    new_reading_available = false;
    attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoPinISR, CHANGE);
#endif
}
