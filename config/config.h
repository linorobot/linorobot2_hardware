#ifdef USE_BEEBO_CONFIG
    #include "custom/beebo_config.h"
#endif

#ifdef USE_HEX_CONFIG
    #include "custom/hex_config.h"
#endif

#ifdef USE_DEV_CONFIG
    #include "custom/dev_config.h"
#endif

#if !defined (USE_BEEBO_CONFIG) && !defined (USE_DEV_CONFIG) && !defined (USE_HEX_CONFIG)
    #include "lino_base_config.h"
#endif

