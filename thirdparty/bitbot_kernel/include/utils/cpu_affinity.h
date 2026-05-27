#ifndef _CPU_AFFINITY_H
#define _CPU_AFFINITY_H

#include <stdbool.h>
//#include <bits/pthreadtypes.h>

#ifdef __cplusplus
extern "C"
{
#endif // __cplusplus

#ifdef __linux__
bool SetCpuAffinity();
bool StickThisThreadToCore(int core_id);
bool StickThisThreadToCores(int* cores_id, int size);
#endif // __linux__

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _CPU_AFFINITY_H
