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

#ifndef MOTOR_INTERFACE
#define MOTOR_INTERFACE

class MotorInterface
{
    bool invert_;
    protected:
        virtual void forward(int pwm) = 0;
        virtual void reverse(int pwm) = 0;

    public:
        MotorInterface(int invert):
            invert_(invert)
        {
        }

        virtual void brake() = 0;
        void spin(int pwm)
        {
            if(invert_)
                pwm *= -1;

            if(pwm > 0)
                forward(pwm);
            else if(pwm < 0)
                reverse(pwm);
            else
                brake();
        }
};

#endif