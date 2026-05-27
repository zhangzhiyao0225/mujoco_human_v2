#ifndef _PRIORITY_H
#define _PRIORITY_H

#include <stdbool.h>


#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#ifdef __linux__

bool setProcessHighPriority(unsigned int priority);

bool SetDeadlinePolicy(unsigned int runtime, unsigned int period, unsigned int deadline);
#endif // __linux__

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _PRIORITY_H