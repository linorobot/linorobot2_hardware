#include <Arduino.h>
#include <sensor_msgs/msg/battery_state.h>
#include "config.h"

#ifdef BATTERY_PIN
double readVoltage(int pin) {
  int reading = 0;
  int i;
  for (i = 0; i < 4; i++) // smoothing
    reading += analogRead(pin);
  reading /= i;
  return BATTERY_ADJUST(reading);
}
#endif

sensor_msgs__msg__BatteryState battery_msg_;
sensor_msgs__msg__BatteryState getBattery()
{
#ifdef BATTERY_PIN
    battery_msg_.voltage = readVoltage(BATTERY_PIN);
#endif
    return battery_msg_;
}
