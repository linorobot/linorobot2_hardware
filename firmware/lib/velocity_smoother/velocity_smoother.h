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

#ifndef VELOCITY_SMOOTHER
#define VELOCITY_SMOOTHER


class VelocitySmoother
{
    public:
        VelocitySmoother(float beta=0.01):
            prev_vel_(0.0),
            alpha_(0.0),
            beta_(0.0)
        {
            beta_ = beta;
            alpha_ = 1.0 - beta_;
        }

        float smooth(float val)
        {
            prev_vel_ = (val * beta_) + (prev_vel_ * alpha_);
            return prev_vel_;
        }

    private:
        float prev_vel_;
        float alpha_;
        float beta_;
};

#endif
