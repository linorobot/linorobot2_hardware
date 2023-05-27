#include <Arduino.h>
#include <sensor_msgs/msg/range.h>
#include "config.h"

//define sound speed in m/uS
#define SOUND_SPEED 0.00034
#define TIMEOUT 5000 // uS
#define FOV (15 * 0.0174533) // field of view rad
#define MIN_RANGE 0.02 // 2cm
#define MAX_RANGE (SOUND_SPEED * TIMEOUT / 2 * 0.95)
// the 5000uS timeout limits range to roughly 0.8m

sensor_msgs__msg__Range range_msg_;
sensor_msgs__msg__Range getRange()
{
#ifdef TRIG_PIN // ultrasonic sensor HC-SR04
    unsigned long duration;
    // ping
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(ECHO_PIN, HIGH, TIMEOUT);
    range_msg_.range = duration ? (duration * SOUND_SPEED / 2) : +INFINITY;
    range_msg_.field_of_view = FOV;
    range_msg_.min_range = MIN_RANGE;
    range_msg_.max_range = MAX_RANGE;
#endif
    return range_msg_;
}

void initRange()
{
#ifdef TRIG_PIN // ultrasonic sensor HC-SR04
    pinMode(TRIG_PIN, OUTPUT);
#endif
}
