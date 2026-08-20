#include <Arduino.h>
#include "config.h"

#ifdef USE_SYSLOG
#include <WiFiUdp.h>
#include <Syslog.h>
#ifndef DEVICE_HOSTNAME
#define DEVICE_HOSTNAME "linorobot2"
#endif
#ifndef APP_NAME
#define APP_NAME "hardware"
#endif
WiFiUDP udpClient;
Syslog syslogv(udpClient, SYSLOG_SERVER, SYSLOG_PORT, DEVICE_HOSTNAME, APP_NAME, LOG_KERN);
void syslog(uint16_t priority, const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  syslogv.vlogf(priority, fmt, args);
  va_end(args);
};
#endif
