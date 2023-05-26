#include <Arduino.h>
#include <sensor_msgs/msg/battery_state.h>
#include "config.h"

#ifdef USE_INA219
#include <INA219_WE.h>

#define INA219_ADDRESS 0x42
INA219_WE ina219 = INA219_WE(INA219_ADDRESS);

float shuntVoltage_mV = 0.0;
float loadVoltage_V = 0.0;
float busVoltage_V = 0.0;
float current_mA = 0.0;
float power_mW = 0.0;
bool ina219_overflow = false;

void initBattery(){
  if(!ina219.init()){
    Serial.println("INA219 not connected!");
  }
  ina219.setADCMode(BIT_MODE_9);
  ina219.setPGain(PG_320);
  ina219.setBusRange(BRNG_16);
  ina219.setShuntSizeInOhms(0.01); // used in INA219.
}

void InaDataUpdate(){
  shuntVoltage_mV = ina219.getShuntVoltage_mV();
  busVoltage_V = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getBusPower();
  loadVoltage_V  = busVoltage_V + (shuntVoltage_mV/1000);
  ina219_overflow = ina219.getOverflow();
}
#else
void initBattery() {};
#endif

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
#ifdef USE_INA219
    // read voltage
    InaDataUpdate();
    battery_msg_.voltage = loadVoltage_V;
    battery_msg_.current = -current_mA / 1000; // Amp, minu when discharge
#endif
    return battery_msg_;
}
