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
#include <Arduino.h>
#include "config.h"
#include "syslog.h"
#include "wifis.h"

#ifdef WIFI_AP_LIST
#include <WiFi.h>
#include <WiFiMulti.h>
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile unsigned long init = millis(); \
  if (millis() - init > MS) { X; init = millis();} \
} while (0)

const char *wifi_ap_list[][2] = WIFI_AP_LIST;
WiFiMulti wifiMulti;

void initWifis(void)
{
    for (int i = 0; wifi_ap_list[i][0] != NULL; i++) {
        wifiMulti.addAP(wifi_ap_list[i][0], wifi_ap_list[i][1]);
    }
    while (wifiMulti.run() != WL_CONNECTED) {
        delay(500);
    }
    Serial.println("WIFI connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    syslog(LOG_INFO, "%s ssid %s rssi %d ip %s", __FUNCTION__, WiFi.SSID(), WiFi.RSSI(),
	   WiFi.localIP().toString().c_str());
}

void runWifis(void)
{
    static uint8_t dis_bssid[6]; // check previous disconnected bssid to avoid repeated disconnection
    uint8_t *bssid;
    uint8_t bssidv[6];

#ifdef WIFI_MONITOR
    EXECUTE_EVERY_N_MS(WIFI_MONITOR * 60 * 1000, syslog(LOG_INFO, "%s ssid %s rssi %d", \
							__FUNCTION__, WiFi.SSID(), WiFi.RSSI()));
#endif
#ifdef PICO // WiFi.BSSID api is different
    // when wifi signal is too weak, disconnect current ap and scan for strongest signal
    EXECUTE_EVERY_N_MS(2000, (WiFi.RSSI() < LOW_RSSI && (bssid = WiFi.BSSID(bssidv), memcmp(dis_bssid, bssid, 6))) ? \
		       (memcpy(dis_bssid, bssid, 6), WiFi.disconnect()) : 0);
#else
    // when wifi signal is too weak, disconnect current ap and scan for strongest signal
    EXECUTE_EVERY_N_MS(2000, (WiFi.RSSI() < LOW_RSSI && (bssid = WiFi.BSSID(), memcmp(dis_bssid, bssid, 6))) ? \
		       (memcpy(dis_bssid, bssid, 6), WiFi.disconnect()) : 0);
#endif
    wifiMulti.run();
}
#endif // WIFI_AP_LIST
