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

#ifndef DEFAULT_MOTOR
#define DEFAULT_MOTOR

#include <Arduino.h>
#ifdef ESP32
#include <ESP32Servo.h>
#else
#include <Servo.h>
#endif
#include "config.h"
#ifdef ESP32
inline void analogWriteFrequency(uint8_t pin, double frequency)
{
  analogWriteFrequency(frequency);
}
#elif defined(PICO)
inline void analogWriteFrequency(double frequency)
{
  analogWriteFreq(frequency);
}
inline void analogWriteFrequency(uint8_t pin, double frequency)
{
  analogWriteFreq(frequency);
}
#endif

#include "motor_interface.h"

class Generic2: public MotorInterface
{
    private:
        int in_a_pin_;
        int in_b_pin_;
        int pwm_pin_;

    protected:
        void forward(int pwm) override
        {
            if (in_a_pin_ < 0) return;
            digitalWrite(in_a_pin_, HIGH);
            digitalWrite(in_b_pin_, LOW);
            analogWrite(pwm_pin_, abs(pwm));
        }

        void reverse(int pwm) override
        {
            if (in_a_pin_ < 0) return;
            digitalWrite(in_a_pin_, LOW);
            digitalWrite(in_b_pin_, HIGH);
            analogWrite(pwm_pin_, abs(pwm));
        }

    public:
        Generic2(float pwm_frequency, int pwm_bits, bool invert, int pwm_pin, int in_a_pin, int in_b_pin): 
            MotorInterface(invert),
            in_a_pin_(in_a_pin),
            in_b_pin_(in_b_pin),
            pwm_pin_(pwm_pin)
        {
            if (in_a_pin_ < 0) return;
            pinMode(in_a_pin_, OUTPUT);
            pinMode(in_b_pin_, OUTPUT);
            pinMode(pwm_pin_, OUTPUT);

            if(pwm_frequency > 0)
            {
                analogWriteFrequency(pwm_pin_, pwm_frequency);
            }
            analogWriteResolution(pwm_bits);

            //ensure that the motor is in neutral state during bootup
            analogWrite(pwm_pin_, abs(0));
        }

        void brake() override
        {
            if (in_a_pin_ < 0) return;
            analogWrite(pwm_pin_, 0);
#ifdef USE_SHORT_BRAKE
            digitalWrite(in_a_pin_, HIGH); // short brake
            digitalWrite(in_b_pin_, HIGH);
#endif
        }
};

class Generic1: public MotorInterface
{
    private:
        int in_pin_;
        int pwm_pin_;

    protected:
        void forward(int pwm) override
        {
            if (in_pin_ < 0) return;
            digitalWrite(in_pin_, HIGH);
            analogWrite(pwm_pin_, abs(pwm));
        }

        void reverse(int pwm) override
        {
            if (in_pin_ < 0) return;
            digitalWrite(in_pin_, LOW);
            analogWrite(pwm_pin_, abs(pwm));
        }

    public:
        Generic1(float pwm_frequency, int pwm_bits, bool invert, int pwm_pin, int in_pin, int unused=-1): 
            MotorInterface(invert),
            in_pin_(in_pin),
            pwm_pin_(pwm_pin)
        {
            if (in_pin_ < 0) return;
            pinMode(in_pin_, OUTPUT);
            pinMode(pwm_pin_, OUTPUT);

            if(pwm_frequency > 0)
            {
                analogWriteFrequency(pwm_pin_, pwm_frequency);
            }
            analogWriteResolution(pwm_bits);

            //ensure that the motor is in neutral state during bootup
            analogWrite(pwm_pin_, abs(0));
        }

        void brake() override
        {
            if (in_pin_ < 0) return;
            analogWrite(pwm_pin_, 0);
        }
};

class BTS7960: public MotorInterface
{
    private:
        int in_a_pin_;
        int in_b_pin_;
        int pwm_max_;

    protected:
        void forward(int pwm) override
        {
            if (in_a_pin_ < 0) return;
#ifdef USE_SHORT_BRAKE
            analogWrite(in_a_pin_, pwm_max_ - abs(pwm));
            analogWrite(in_b_pin_, pwm_max_); // short brake
#else
            analogWrite(in_a_pin_, 0);
            analogWrite(in_b_pin_, abs(pwm));
#endif
        }

        void reverse(int pwm) override
        {
            if (in_a_pin_ < 0) return;
#ifdef USE_SHORT_BRAKE
            analogWrite(in_b_pin_, pwm_max_ - abs(pwm));
            analogWrite(in_a_pin_, pwm_max_); // short brake
#else
            analogWrite(in_b_pin_, 0);
            analogWrite(in_a_pin_, abs(pwm));
#endif
        }

    public:
        BTS7960(float pwm_frequency, int pwm_bits, bool invert, int unused, int in_a_pin, int in_b_pin): 
            MotorInterface(invert),
            in_a_pin_(in_a_pin),
            in_b_pin_(in_b_pin)
        {
            if (in_a_pin_ < 0) return;
            pwm_max_ = (1 << pwm_bits) - 1;
            pinMode(in_a_pin_, OUTPUT);
            pinMode(in_b_pin_, OUTPUT);

            if(pwm_frequency > 0)
            {
                analogWriteFrequency(in_a_pin_, pwm_frequency);
                analogWriteFrequency(in_b_pin_, pwm_frequency);

            }
            analogWriteResolution(pwm_bits);

            //ensure that the motor is in neutral state during bootup
            analogWrite(in_a_pin_, 0);
            analogWrite(in_b_pin_, 0);
        }
    
        BTS7960(float pwm_frequency, int pwm_bits, bool invert, int in_a_pin, int in_b_pin): 
            MotorInterface(invert),
            in_a_pin_(in_a_pin),
            in_b_pin_(in_b_pin)
        {
            if (in_a_pin_ < 0) return;
            pwm_max_ = (1 << pwm_bits) - 1;
            pinMode(in_a_pin_, OUTPUT);
            pinMode(in_b_pin_, OUTPUT);

            if(pwm_frequency > 0)
            {
                analogWriteFrequency(in_a_pin_, pwm_frequency);
                analogWriteFrequency(in_b_pin_, pwm_frequency);

            }
            analogWriteResolution(pwm_bits);

            //ensure that the motor is in neutral state during bootup
            analogWrite(in_a_pin_, 0);
            analogWrite(in_b_pin_, 0);
        }

        void brake() override
        {
            if (in_a_pin_ < 0) return;
#ifdef USE_SHORT_BRAKE
            analogWrite(in_a_pin_, pwm_max_);
            analogWrite(in_b_pin_, pwm_max_); // short brake
#else
            analogWrite(in_b_pin_, 0);
            analogWrite(in_a_pin_, 0);            
#endif
        }
};

class ESC: public MotorInterface
{
    private:
        Servo motor_;
        int pwm_pin_;

    protected:
        void forward(int pwm) override
        {
            if (pwm_pin_ < 0) return;
            motor_.writeMicroseconds(1500 + pwm);
        }

        void reverse(int pwm) override
        {
            if (pwm_pin_ < 0) return;
            motor_.writeMicroseconds(1500 + pwm);
        }

    public:
        ESC(float pwm_frequency, int pwm_bits, bool invert, int pwm_pin, int unused=-1, int unused2=-1): 
            MotorInterface(invert),
            pwm_pin_(pwm_pin)
        {
            if (pwm_pin_ < 0) return;
            motor_.attach(pwm_pin);
            
            //ensure that the motor is in neutral state during bootup
            motor_.writeMicroseconds(1500);
        }

        void brake() override
        {
            if (pwm_pin_ < 0) return;
            motor_.writeMicroseconds(1500);         
        }
};

#endif