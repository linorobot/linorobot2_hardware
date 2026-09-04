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

#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "env.h"

#if defined(USE_BMP280)

#ifndef BMP280_ADDR
#define BMP280_ADDR 0x77          // Waveshare General Driver board barometer
#endif

// Registers (BMP280 / BME280, Bosch datasheets)
#define REG_CALIB_00   0x88       // dig_T1..dig_P9   (24 bytes)
#define REG_CALIB_H1   0xA1       // dig_H1           (BME280)
#define REG_ID         0xD0
#define REG_RESET      0xE0
#define REG_CALIB_H2   0xE1       // dig_H2..dig_H6   (7 bytes, BME280)
#define REG_CTRL_HUM   0xF2       // BME280 only
#define REG_STATUS     0xF3
#define REG_CTRL_MEAS  0xF4
#define REG_CONFIG     0xF5
#define REG_PRESS_MSB  0xF7       // press[3] temp[3] (hum[2] on BME280)

#define CHIP_BMP280    0x58
#define CHIP_BME280    0x60

static uint8_t  s_addr   = 0;
static bool     s_ok     = false;
static bool     s_is_bme = false;

// temperature / pressure trimming parameters
static uint16_t dig_T1;
static int16_t  dig_T2, dig_T3;
static uint16_t dig_P1;
static int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
// humidity trimming parameters (BME280)
static uint8_t  dig_H1, dig_H3;
static int16_t  dig_H2, dig_H4, dig_H5;
static int8_t   dig_H6;

static bool readRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    Wire.beginTransmission(s_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0)
        return false;
    if (Wire.requestFrom((int)s_addr, (int)len) != len)
        return false;
    for (uint8_t i = 0; i < len; i++)
        buf[i] = Wire.read();
    return true;
}

static bool writeReg(uint8_t reg, uint8_t val)
{
    Wire.beginTransmission(s_addr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission() == 0;
}

static uint8_t readReg8(uint8_t reg)
{
    uint8_t v = 0;
    readRegs(reg, &v, 1);
    return v;
}

static bool probe(uint8_t addr)
{
    s_addr = addr;
    uint8_t id = readReg8(REG_ID);
    if (id == CHIP_BME280) { s_is_bme = true;  return true; }
    if (id == CHIP_BMP280) { s_is_bme = false; return true; }
    return false;
}

static void loadCalibration()
{
    uint8_t b[24];
    readRegs(REG_CALIB_00, b, 24);
    dig_T1 = (uint16_t)(b[1]  << 8 | b[0]);
    dig_T2 = (int16_t) (b[3]  << 8 | b[2]);
    dig_T3 = (int16_t) (b[5]  << 8 | b[4]);
    dig_P1 = (uint16_t)(b[7]  << 8 | b[6]);
    dig_P2 = (int16_t) (b[9]  << 8 | b[8]);
    dig_P3 = (int16_t) (b[11] << 8 | b[10]);
    dig_P4 = (int16_t) (b[13] << 8 | b[12]);
    dig_P5 = (int16_t) (b[15] << 8 | b[14]);
    dig_P6 = (int16_t) (b[17] << 8 | b[16]);
    dig_P7 = (int16_t) (b[19] << 8 | b[18]);
    dig_P8 = (int16_t) (b[21] << 8 | b[20]);
    dig_P9 = (int16_t) (b[23] << 8 | b[22]);

    if (s_is_bme)
    {
        uint8_t h[7];
        dig_H1 = readReg8(REG_CALIB_H1);
        readRegs(REG_CALIB_H2, h, 7);
        dig_H2 = (int16_t)(h[1] << 8 | h[0]);
        dig_H3 = h[2];
        dig_H4 = (int16_t)((int8_t)h[3] * 16 | (h[4] & 0x0F));
        dig_H5 = (int16_t)((int8_t)h[5] * 16 | (h[4] >> 4));
        dig_H6 = (int8_t)h[6];
    }
}

bool initEnv()
{
    s_ok = false;
    s_is_bme = false;

    if (!probe(BMP280_ADDR))
    {
        uint8_t alt = (BMP280_ADDR == 0x76) ? 0x77 : 0x76;
        if (!probe(alt))
            return false;
    }

    writeReg(REG_RESET, 0xB6);           // soft reset
    delay(5);
    // re-verify id after reset (also re-selects s_is_bme)
    if (!probe(s_addr))
        return false;

    loadCalibration();

    // NORMAL mode so the sensor free-runs — readEnv() is then a single burst
    // register read with NO delay()/polling, and never stalls the 50 Hz
    // control loop it is called from.
    //   config   : t_sb = 1000 ms standby (0b101<<5), IIR filter x4 (0b100<<2)
    //   ctrl_hum : humidity oversampling x1 (BME280, write before ctrl_meas)
    //   ctrl_meas: osrs_t x1, osrs_p x1, mode = normal (0b11)
    writeReg(REG_CONFIG, (0b101 << 5) | (0b100 << 2));
    if (s_is_bme)
        writeReg(REG_CTRL_HUM, 0x01);
    writeReg(REG_CTRL_MEAS, (0x01 << 5) | (0x01 << 2) | 0x03);

    s_ok = true;
    return true;
}

bool envOk()          { return s_ok; }
bool envHasHumidity() { return s_ok && s_is_bme; }

EnvData readEnv()
{
    EnvData d = { false, 0.0f, 0.0f, 0.0f };
    if (!s_ok)
        return d;

    // Non-blocking: the sensor is in normal mode, just read the latest result.
    uint8_t b[8];
    uint8_t len = s_is_bme ? 8 : 6;
    if (!readRegs(REG_PRESS_MSB, b, len))
        return d;

    int32_t adc_P = ((int32_t)b[0] << 12) | ((int32_t)b[1] << 4) | (b[2] >> 4);
    int32_t adc_T = ((int32_t)b[3] << 12) | ((int32_t)b[4] << 4) | (b[5] >> 4);

    // --- temperature (also yields t_fine), Bosch float formula ---
    double v1 = (((double)adc_T) / 16384.0 - ((double)dig_T1) / 1024.0) * ((double)dig_T2);
    double v2 = ((((double)adc_T) / 131072.0 - ((double)dig_T1) / 8192.0) *
                (((double)adc_T) / 131072.0 - ((double)dig_T1) / 8192.0)) * ((double)dig_T3);
    double t_fine = v1 + v2;
    d.temperature = (float)(t_fine / 5120.0);

    // --- pressure ---
    v1 = (t_fine / 2.0) - 64000.0;
    v2 = v1 * v1 * ((double)dig_P6) / 32768.0;
    v2 = v2 + v1 * ((double)dig_P5) * 2.0;
    v2 = (v2 / 4.0) + (((double)dig_P4) * 65536.0);
    v1 = (((double)dig_P3) * v1 * v1 / 524288.0 + ((double)dig_P2) * v1) / 524288.0;
    v1 = (1.0 + v1 / 32768.0) * ((double)dig_P1);
    if (v1 != 0.0)
    {
        double p = 1048576.0 - (double)adc_P;
        p = (p - (v2 / 4096.0)) * 6250.0 / v1;
        v1 = ((double)dig_P9) * p * p / 2147483648.0;
        v2 = p * ((double)dig_P8) / 32768.0;
        p = p + (v1 + v2 + ((double)dig_P7)) / 16.0;
        d.pressure = (float)p;                 // Pascals
    }

    // --- humidity (BME280 only), reported as fraction 0..1 (REP-145) ---
    if (s_is_bme)
    {
        int32_t adc_H = ((int32_t)b[6] << 8) | b[7];
        double h = t_fine - 76800.0;
        h = ((double)adc_H - (((double)dig_H4) * 64.0 + ((double)dig_H5) / 16384.0 * h)) *
            (((double)dig_H2) / 65536.0 * (1.0 + ((double)dig_H6) / 67108864.0 * h *
             (1.0 + ((double)dig_H3) / 67108864.0 * h)));
        h = h * (1.0 - ((double)dig_H1) * h / 524288.0);
        if (h > 100.0) h = 100.0;
        else if (h < 0.0) h = 0.0;
        d.humidity = (float)(h / 100.0);
    }

    d.valid = true;
    return d;
}

#else  // !USE_BMP280 — stubs so the firmware links without the sensor

bool    initEnv()        { return false; }
bool    envOk()          { return false; }
bool    envHasHumidity() { return false; }
EnvData readEnv()        { EnvData d = { false, 0.0f, 0.0f, 0.0f }; return d; }

#endif
