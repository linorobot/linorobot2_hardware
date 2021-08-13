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
#define ENCODER_OPTIMIZE_INTERRUPTS
#include "encoder.h"
#include "kinematics.h"

Encoder motor1_encoder(MOTOR1_ENCODER_A, MOTOR1_ENCODER_B, 5, MOTOR1_ENCODER_INV);
Encoder motor2_encoder(MOTOR2_ENCODER_A, MOTOR2_ENCODER_B, 5, MOTOR2_ENCODER_INV);
Encoder motor3_encoder(MOTOR3_ENCODER_A, MOTOR3_ENCODER_B, 5, MOTOR3_ENCODER_INV);
Encoder motor4_encoder(MOTOR4_ENCODER_A, MOTOR4_ENCODER_B, 5, MOTOR4_ENCODER_INV);

Motor motor1_controller(MOTOR1_INV, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2_controller(MOTOR2_INV, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);
Motor motor3_controller(MOTOR3_INV, MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);
Motor motor4_controller(MOTOR4_INV, MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B);

void setup()
{
    Serial.begin(115200);
    unsigned long start_time = micros();
    while(true)
    {
        if(micros() - start_time >= 10000000)
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
}

void loop()
{
    delay(100);
    Serial.print("M1: ");
    Serial.print(motor1_encoder.read());

    Serial.print(" M2: ");
    Serial.print(motor2_encoder.read());

    Serial.print(" M3: ");
    Serial.print(motor3_encoder.read());

    Serial.print(" M4: ");
    Serial.println(motor4_encoder.read());
}
