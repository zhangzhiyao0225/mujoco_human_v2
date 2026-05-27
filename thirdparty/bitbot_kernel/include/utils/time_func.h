#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#ifdef __linux__

#include <unistd.h>

struct timespec DiffTime(struct timespec start, struct timespec end);

void SleepUS(uint32_t us);
void SleepNS(uint32_t ns);
  
#endif // __linux__

void SleepS(uint32_t s);
void SleepMS(uint32_t ms);

#ifdef __cplusplus
}
#endif // __cplusplus
