#ifndef BATTERY_H
#define BATTERY_H

#include <sensor_msgs/msg/battery_state.h>
sensor_msgs__msg__BatteryState getBattery();
void initBattery();
void getBatteryPercentage(sensor_msgs__msg__BatteryState *msg);

#endif
