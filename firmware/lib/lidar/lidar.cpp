// Copyright (c) 2023 Thomas Chou
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
//
// Push lidar data to a UDP server, which will push laser scan message
// TODO: add PWM closed loop control the scan frequency
// TODO: add TCP for reliable data packet sequence
//
#include <Arduino.h>
#include "config.h"
#include "syslog.h"

#ifdef USE_LIDAR_UDP
#include <HardwareSerial.h>
#include <WiFiUdp.h>
#define BUFSIZE 512

HardwareSerial comm(LIDAR_SERIAL);
WiFiUDP udp;
uint8_t buf[BUFSIZE];

void rx_err_callback(hardwareSerial_error_t err)
{
  syslog(LOG_INFO, "%s err %d", __FUNCTION__, err);
}

size_t len = 0;
void rx_callback(void)
{
  size_t cc = comm.read(buf + len, BUFSIZE - len);
  // syslog(LOG_INFO, "%s recv %d", __FUNCTION__, cc);
  // send larger UDP packet
  if ((len += cc) && (len > 128)) {
      udp.beginPacket(LIDAR_SERVER, LIDAR_PORT);
      udp.write(buf, len);
      udp.endPacket();
      // syslog(LOG_INFO, "%s send %d", __FUNCTION__, len);
      len = 0;
  }
}

void poweronLidar(void)
{
#ifdef LIDAR_POWEROFF
    digitalWrite(LIDAR_POWEROFF, LOW);
#endif
}

void poweroffLidar(void)
{
#ifdef LIDAR_POWEROFF
    digitalWrite(LIDAR_POWEROFF, HIGH);
#endif
}

void initLidar(void) {
  pinMode(LIDAR_RXD, INPUT);
#ifdef LIDAR_POWEROFF
  pinMode(LIDAR_POWEROFF, OUTPUT);
#endif
  poweronLidar();
  comm.setRxBufferSize(1024);
  comm.onReceiveError(rx_err_callback);
  comm.onReceive(rx_callback);
  comm.begin(LIDAR_BAUDRATE, SERIAL_8N1, LIDAR_RXD);
};
#else
void initLidar(void) {};
void poweronLidar(void) {};
void poweroffLidar(void) {};
#endif
