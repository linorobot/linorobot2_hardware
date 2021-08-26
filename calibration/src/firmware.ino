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
#include "config.h"
#include "motor.h"
#define ENCODER_USE_INTERRUPTS
#include "encoder.h"
#include "kinematics.h"

#define SAMPLE_TIME 10 //s

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, 5, MOTOR1_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, 5, MOTOR2_ENCODER_INV);
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, 5, MOTOR3_ENCODER_INV);
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, 5, MOTOR4_ENCODER_INV);

Motor motor1_controller(MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2_controller(MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
Motor motor3_controller(MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Motor motor4_controller(MOTOR4_INV, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

long long int counts_per_rev1;
long long int counts_per_rev2;
long long int counts_per_rev3;
long long int counts_per_rev4;

void setup()
{
    Serial.begin(115200);
    unsigned long start_time = micros();
    while(true)
    {
        if(micros() - start_time >= SAMPLE_TIME * 1000000)
        {
            motor1_controller.spin(0);
            motor2_controller.spin(0);
            motor3_controller.spin(0);
            motor4_controller.spin(0);
            break;
        }
        motor1_controller.spin(PWM_MAX);
        motor2_controller.spin(PWM_MAX);
        motor3_controller.spin(PWM_MAX);
        motor4_controller.spin(PWM_MAX);
    }
    float measured_voltage = constrain(MOTOR_POWER_MEASURED_VOLTAGE, 0, MOTOR_OPERATING_VOLTAGE);
    int scaled_max_rpm = ((measured_voltage / MOTOR_OPERATING_VOLTAGE) * MOTOR_MAX_RPM);
    int total_rev = scaled_max_rpm * (SAMPLE_TIME / 60.0);

    counts_per_rev1 = motor1_encoder.read() / total_rev;
    counts_per_rev2 = motor2_encoder.read() / total_rev;
    counts_per_rev3 = motor3_encoder.read() / total_rev;
    counts_per_rev4 = motor4_encoder.read() / total_rev;
}

void loop()
{
    delay(1500);
    Serial.println("================MOTOR ENCODER READINGS================");

    Serial.print("FRONT LEFT - M1: ");
    Serial.print(motor1_encoder.read());

    Serial.print(" FRONT RIGHT - M2: ");
    Serial.println(motor2_encoder.read());

    Serial.print("BACK LEFT - M3: ");
    Serial.print(motor3_encoder.read());

    Serial.print(" BACK RIGHT - M4: ");
    Serial.println(motor4_encoder.read());
    Serial.println();

    Serial.println("================COUNTS PER REVOLUTION=================");
    Serial.print("FRONT LEFT - CPR1: ");
    Serial.print(counts_per_rev1);

    Serial.print(" FRONT RIGHT - CPR2: ");
    Serial.println(counts_per_rev2);

    Serial.print("BACK LEFT - CPR3: ");
    Serial.print(counts_per_rev3);

    Serial.print(" BACK RIGHT - CPR4: ");
    Serial.println(counts_per_rev4);
    Serial.println();
}
