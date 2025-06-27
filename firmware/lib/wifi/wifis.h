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
#ifndef WIFIS_H
#define WIFIS_H

#if defined(USE_WIFI_TRANSPORT) && !defined(WIFI_AP_LIST) // fixup old wifi config
#define WIFI_AP_LIST {{WIFI_SSID, WIFI_PASSWORD}, {NULL}}
#endif
#ifndef LOW_RSSI
#define LOW_RSSI -75 // when wifi signal is too low, disconnect current ap and scan for strongest signal
#endif

#ifdef WIFI_AP_LIST
void initWifis(void);
void runWifis(void);
#else
#define initWifis()
#define runWifis()
#endif

#endif
