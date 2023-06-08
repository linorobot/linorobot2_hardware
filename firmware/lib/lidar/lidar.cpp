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
#define RX_BUFSIZE 2048
#define BUFSIZE 1460

HardwareSerial comm(LIDAR_SERIAL);
WiFiUDP udp;
uint8_t buf[BUFSIZE];

void initLidar(void) {
  pinMode(LIDAR_RXD, INPUT);
#ifdef LIDAR_PWM
  pinMode(LIDAR_PWM, OUTPUT);
  analogWrite(LIDAR_PWM, 1 << PWM_BITS -1);
#endif
  comm.begin(LIDAR_BAUDRATE, SERIAL_8N1, LIDAR_RXD);
  comm.setRxBufferSize(RX_BUFSIZE);
};

size_t len = 0;
void pushLidar(void) {
  // send larger UDP packet
  if ((len += comm.read(buf + len, BUFSIZE - len)) && (len > 800)) {
      udp.beginPacket(LIDAR_SERVER, LIDAR_PORT);
      udp.write(buf, len);
      udp.endPacket();
      // syslog(LOG_INFO, "%s send %lu", __FUNCTION__, len);
      len = 0;
  }
};

#else
void initLidar(void) {};
void pushLidar(void) {};
#endif
