#ifndef SYSLOG_H_
#define SYSLOG_H_ 

#include <Syslog.h>
#include "config.h"

#ifdef USE_SYSLOG
void syslog(uint16_t priority, const char *fmt, ...);
#else
#define syslog(...)
#endif

#endif
