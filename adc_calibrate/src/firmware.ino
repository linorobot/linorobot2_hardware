// Copyright (c) 2024 Thomas Chou, Linorobot contributors
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

/**
 * ADC Calibration Utility
 * Builds an ESP32 ADC Lookup Table (LUT) to linearize ADC readings.
 *
 * Based on original work from Helmut Weber & Henry Cheung.
 */

#include <Arduino.h>

#if defined(ESP32) && !defined(CONFIG_IDF_TARGET_ESP32S3) && !defined(CONFIG_IDF_TARGET_ESP32C3) && !defined(CONFIG_IDF_TARGET_ESP32C6)
#include <driver/dac.h>
#define HAS_HARDWARE_DAC 1
#endif

#include "config.h"

#ifndef BAUDRATE
#define BAUDRATE 115200
#endif

#ifndef BATTERY_PIN
#define BATTERY_PIN 33
#endif

#define ADC_PIN BATTERY_PIN

float Results[4097];
float Res2[4096 * 5];

void dumpResults() {
    for (int i = 0; i < 4096; i++) {
        if (i % 16 == 0) {
            Serial.println();
            Serial.print(i);
            Serial.print(" - ");
        }
        Serial.print(Results[i], 2);
        Serial.print(", ");
    }
    Serial.println();
}

void dumpRes2() {
    Serial.println(F("Dump Res2 data..."));
    for (int i = 0; i < (5 * 4096); i++) {
        if (i % 16 == 0) {
            Serial.println();
            Serial.print(i);
            Serial.print(" - ");
        }
        Serial.print(Res2[i], 3);
        Serial.print(", ");
    }
    Serial.println();
}

void setup() {
#ifdef HAS_HARDWARE_DAC
    dac_output_enable(DAC_CHANNEL_1); // pin 25
    dac_output_voltage(DAC_CHANNEL_1, 0);
    analogReadResolution(12);
#endif
    Serial.begin(BAUDRATE);
    delay(1000);
}

void loop() {
#ifdef HAS_HARDWARE_DAC
    Serial.print(F("Test Linearity "));
    for (int j = 0; j < 500; j++) {
        if (j % 100 == 0) Serial.print(".");
        for (int i = 0; i < 256; i++) {
            dac_output_voltage(DAC_CHANNEL_1, (i & 0xff));
            delayMicroseconds(100);
            Results[i * 16] = 0.9 * Results[i * 16] + 0.1 * analogRead(ADC_PIN);
        }
    }
    Serial.println();

    Serial.println(F("Calculate interpolated values .."));
    Results[4096] = 4095.0;
    for (int i = 0; i < 256; i++) {
        for (int j = 1; j < 16; j++) {
            Results[i * 16 + j] = Results[i * 16] + (Results[(i + 1) * 16] - Results[i * 16]) * (float)j / 16.0f;
        }
    }

    Serial.println(F("Generating LUT .."));
    for (int i = 0; i < 4096; i++) {
        Results[i] = 0.5f + Results[i];
    }

    Results[4096] = 4095.5000;
    for (int i = 0; i < 4096; i++) {
        for (int j = 0; j < 5; j++) {
            Res2[i * 5 + j] = Results[i] + (Results[(i + 1)] - Results[i]) * (float)j / 10.0f;
        }
    }

    for (int i = 1; i < 4096; i++) {
        int index = 0;
        float minDiff = 99999.0f;
        for (int j = 0; j < (5 * 4096); j++) {
            float diff = fabs((float)i - Res2[j]);
            if (diff < minDiff) {
                minDiff = diff;
                index = j;
            }
        }
        Results[i] = (float)index;
    }

    for (int i = 0; i < 4096; i++) {
        Results[i] /= 5;
    }

    Serial.println();
    Serial.println("const int16_t ADC_LUT[4096] = { 0,");
    for (int i = 1; i < 4095; i++) {
        Serial.print((int)Results[i]);
        Serial.print(",");
        if ((i % 15) == 0) Serial.println();
    }
    Serial.println((int)Results[4095]);
    Serial.println("};");
#else
    Serial.println(F("ADC calibration LUT generation is only supported on ESP32 MCUs with hardware DAC."));
#endif

    while (1) {
        delay(1000);
    }
}
