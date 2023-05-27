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
#ifndef CONFIG_H
#define CONFIG_H

#ifdef USE_BEEBO_CONFIG
    #include "custom/beebo_config.h"
#endif

#ifdef USE_BEEBO_M_CONFIG
    #include "custom/beebo_m_config.h"
#endif

#ifdef USE_SQUARE_CONFIG
    #include "custom/square_config.h"
#endif

#ifdef USE_DEV_CONFIG
    #include "custom/dev_config.h"
#endif

#ifdef USE_GENDRV_CONFIG
    #include "custom/gendrv_config.h"
#endif

// this should be the last one
#ifndef LINO_BASE
    #include "lino_base_config.h"
#endif

#endif
