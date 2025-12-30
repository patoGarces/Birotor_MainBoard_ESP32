#ifndef __UDP_LOGGER_H__
#define __UDP_LOGGER_H__

#include "stdio.h"

void udpLoggerInit(uint16_t port);
int udpVprintf(const char *fmt, va_list args);

#endif