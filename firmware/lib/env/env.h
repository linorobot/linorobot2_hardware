// Copyright (c) 2021 Juan Miguel Jimeno
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

// Minimal in-tree BMP280 / BME280 environmental sensor driver (I2C).
//
// initEnv() probes BMP280_ADDR (default 0x77) and the alternate address,
// verifies the chip id (0x58 = BMP280, 0x56/0x57/0x58 pre-production, 0x60 =
// BME280) and loads the factory trimming parameters. readEnv() runs a single
// forced-mode conversion and returns the compensated values.
#ifndef ENV_H
#define ENV_H

#include <stdbool.h>

struct EnvData
{
    bool  valid;         // true when the last conversion succeeded
    float temperature;   // degrees Celsius
    float pressure;      // Pascals
    float humidity;      // relative humidity 0..1 (BME280 only; 0 otherwise)
};

bool     initEnv();          // probe + calibrate; true when a sensor is present
bool     envOk();            // result of the last initEnv()
bool     envHasHumidity();   // true only for BME280 (chip id 0x60)
EnvData  readEnv();          // forced measurement + compensation

#endif
