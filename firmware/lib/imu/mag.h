// Copyright (c) 2021 Juan Miguel Jimeno
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

#ifndef MAG_CONFIG_H
#define MAG_CONFIG_H

// include the header of your new driver here similar to default_mag.h
#include "default_mag.h"

// now you can create a config constant that you can use in lino_base_config.h
#ifdef USE_HMC5883L_MAG
    // pass your built in class to MAG macro
    #define MAG HMC5883LMAG
#endif

#ifdef USE_AK8963_MAG
    #define MAG AK8963MAG
#endif

#ifdef USE_AK8975_MAG
    #define MAG AK8975MAG
#endif

#ifdef USE_AK09918_MAG
    #define MAG AK09918MAG
#endif

#ifdef USE_QMC5883L_MAG
    #define MAG QMC5883LMAG
#endif

#ifndef MAG // use fake mag when there is no real mag
    #define USE_FAKE_MAG
    #define MAG FakeMAG
#endif

#endif
