// Copyright (c) 2026 Linorobot contributors
// AI-MAINTAINED AUTO-GENERATED FILE. DO NOT MANUALLY EDIT.
// Generator: Linorobot2 AI Sensor Auto-Detection & Probing Engine
//
// Scans the active I2C bus, identifies responding sensor chips via WHO_AM_I
// and identification registers, and streams machine-readable JSON over serial
// to enable 1-click automatic driver selection in the Web UI Studio.

#include <Arduino.h>
#include <Wire.h>
#include <stdio.h>
#include "config.h"

#ifndef BAUDRATE
#define BAUDRATE 921600
#endif

// Forward declarations for I2C register probing
uint8_t readRegister8(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0xFF;
    Wire.requestFrom((int)addr, 1);
    if (Wire.available()) return Wire.read();
    return 0xFF;
}

uint16_t readRegister16BE(uint8_t addr, uint8_t reg) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0xFFFF;
    Wire.requestFrom((int)addr, 2);
    if (Wire.available() >= 2) {
        uint16_t val = ((uint16_t)Wire.read()) << 8;
        val |= Wire.read();
        return val;
    }
    return 0xFFFF;
}

struct DetectedSensor {
    uint8_t addr;
    const char* category; // "imu", "mag", "current", "env"
    const char* model;    // "QMI8658", "MPU6050", "AK09918", "INA219", etc.
    const char* macro;    // "USE_QMI8658_IMU", "USE_AK09918_MAG", "USE_INA219"
    const char* desc;     // Human readable
};

DetectedSensor detected[16];
int num_detected = 0;

void addDetected(uint8_t addr, const char* category, const char* model, const char* macro, const char* desc) {
    if (num_detected < 16) {
        detected[num_detected++] = { addr, category, model, macro, desc };
    }
}

void scanAndIdentify() {
    num_detected = 0;
    Serial.println("\n=======================================================");
    Serial.println("  🤖 Linorobot2 AI I2C Sensor Auto-Detection Engine    ");
    Serial.println("=======================================================");
    Serial.printf("Scanning I2C bus @ 400kHz (SDA:%d, SCL:%d)...\n", SDA_PIN, SCL_PIN);

    uint8_t found_addrs[128];
    int num_found = 0;

    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            found_addrs[num_found++] = addr;
            Serial.printf(" [0x%02X] Device ACK received\n", addr);
        }
    }

    if (num_found == 0) {
        Serial.println("[-] No I2C devices detected on bus.");
    }

    // Identify detected devices via WHO_AM_I registers
    for (int i = 0; i < num_found; i++) {
        uint8_t addr = found_addrs[i];

        // 1. Check QMI8658 / QMI8658C (0x6B or 0x6A)
        if (addr == 0x6B || addr == 0x6A) {
            uint8_t who = readRegister8(addr, 0x00);
            if (who == 0x05) {
                addDetected(addr, "imu", "QMI8658", "USE_QMI8658_IMU", "QMI8658 6-Axis IMU (Acc+Gyr)");
                continue;
            }
        }

        // 2. Check MPU6050 / MPU9250 / MPU6500 (0x68 or 0x69)
        if (addr == 0x68 || addr == 0x69) {
            uint8_t who = readRegister8(addr, 0x75);
            if (who == 0x68) {
                addDetected(addr, "imu", "MPU6050", "USE_MPU6050_IMU", "MPU6050 6-Axis IMU");
                continue;
            } else if (who == 0x71 || who == 0x73) {
                addDetected(addr, "imu", "MPU9250", "USE_MPU9250_IMU", "MPU9250 9-Axis IMU (Acc+Gyr+Mag)");
                continue;
            } else if (who == 0x70) {
                addDetected(addr, "imu", "MPU6500", "USE_MPU6050_IMU", "MPU6500 6-Axis IMU");
                continue;
            }

            // Check ITG3200 (0x68)
            uint8_t itg_who = readRegister8(addr, 0x00);
            if (itg_who == 0x68) {
                addDetected(addr, "imu", "GY85", "USE_GY85_IMU", "ITG3200 Gyroscope (GY85 component)");
                continue;
            }
        }

        // 3. Check BNO085 / BNO080 (0x4A or 0x4B)
        if (addr == 0x4A || addr == 0x4B) {
            addDetected(addr, "imu", "BNO085", "USE_BNO085_IMU", "BNO085/BNO080 9-DOF Robotic IMU");
            continue;
        }

        // 4. Check BNO055 (0x28 or 0x29)
        if (addr == 0x28 || addr == 0x29) {
            uint8_t who = readRegister8(addr, 0x00);
            if (who == 0xA0) {
                addDetected(addr, "imu", "BNO055", "USE_BNO085_IMU", "BNO055 9-DOF Absolute Orientation IMU");
                continue;
            }
        }

        // 5. Check ADXL345 (0x53)
        if (addr == 0x53) {
            uint8_t who = readRegister8(addr, 0x00);
            if (who == 0xE5) {
                addDetected(addr, "imu", "GY85", "USE_GY85_IMU", "ADXL345 Accelerometer (GY85)");
                continue;
            }
        }

        // 6. Check Magnetometers: AK09918 (0x0C), AK8963 (0x0C), AK8975 (0x0C), QMC5883L (0x0D), HMC5883L (0x1E)
        if (addr >= 0x0C && addr <= 0x0F) {
            uint8_t wia2 = readRegister8(addr, 0x01);
            uint8_t wia = readRegister8(addr, 0x00);
            if (wia2 == 0x09 || addr == 0x0C) {
                addDetected(addr, "mag", "AK09918", "USE_AK09918_MAG", "AK09918 3-Axis Precision Magnetometer");
                continue;
            } else if (wia == 0x48) {
                addDetected(addr, "mag", "AK8963", "USE_AK8963_MAG", "AK8963/AK8975 3-Axis Magnetometer");
                continue;
            } else if (addr == 0x0D) {
                addDetected(addr, "mag", "QMC5883L", "USE_QMC5883L_MAG", "QMC5883L 3-Axis Compass");
                continue;
            }
        }

        if (addr == 0x1E) {
            uint8_t id_a = readRegister8(addr, 10);
            uint8_t id_b = readRegister8(addr, 11);
            if (id_a == 'H' && id_b == '4') {
                addDetected(addr, "mag", "HMC5883L", "USE_HMC5883L_MAG", "HMC5883L 3-Axis Digital Compass");
                continue;
            }
        }

        // 7. Check Current / Power Monitors: INA219 (0x40..0x45, Waveshare GenDrv @ 0x42)
        if (addr >= 0x40 && addr <= 0x45) {
            addDetected(addr, "current", "INA219", "USE_INA219", "INA219 High-Side DC Current & Power Sensor");
            continue;
        }

        // 8. Check BMP280 / BME280 (0x76 or 0x77)
        if (addr == 0x76 || addr == 0x77) {
            uint8_t id = readRegister8(addr, 0xD0);
            if (id == 0x58 || id == 0x60) {
                addDetected(addr, "env", "BMP280", "USE_BMP280", "BMP280/BME280 Environmental Barometer");
                continue;
            }
        }
    }

    // Print Human-Readable Table
    Serial.println("\n--- Identified Hardware Matrix ---");
    const char* detected_imu = "NONE";
    const char* detected_mag = "NONE";
    const char* detected_curr = "NONE";

    for (int i = 0; i < num_detected; i++) {
        Serial.printf(" [%d] ADDR: 0x%02X | CATEGORY: %-8s | MODEL: %-10s | MACRO: %-18s | %s\n",
            i + 1, detected[i].addr, detected[i].category, detected[i].model, detected[i].macro, detected[i].desc);
        if (strcmp(detected[i].category, "imu") == 0 && strcmp(detected_imu, "NONE") == 0) detected_imu = detected[i].model;
        if (strcmp(detected[i].category, "mag") == 0 && strcmp(detected_mag, "NONE") == 0) detected_mag = detected[i].model;
        if (strcmp(detected[i].category, "current") == 0 && strcmp(detected_curr, "NONE") == 0) detected_curr = detected[i].model;
    }

    if (num_detected == 0) {
        Serial.println("  (No known sensor signatures recognized)");
    }

    // Emit Machine-Readable JSON stream for Web UI
    Serial.print("\n[I2C_JSON] {");
    Serial.print("\"status\":\"ok\",");
    Serial.printf("\"imu\":\"%s\",", detected_imu);
    Serial.printf("\"mag\":\"%s\",", detected_mag);
    Serial.printf("\"current\":\"%s\",", detected_curr);
    Serial.print("\"devices\":[");
    for (int i = 0; i < num_detected; i++) {
        if (i > 0) Serial.print(",");
        Serial.printf("{\"addr\":\"0x%02x\",\"category\":\"%s\",\"model\":\"%s\",\"macro\":\"%s\",\"desc\":\"%s\"}",
            detected[i].addr, detected[i].category, detected[i].model, detected[i].macro, detected[i].desc);
    }
    Serial.println("]}");
    Serial.println("=======================================================\n");
}

void setup() {
    Serial.begin(BAUDRATE);
#ifdef BOARD_INIT
    BOARD_INIT;
#else
    Wire.begin();
    Wire.setClock(400000);
#endif
    delay(1500);
    scanAndIdentify();
}

void loop() {
    delay(5000);
    scanAndIdentify();
}
