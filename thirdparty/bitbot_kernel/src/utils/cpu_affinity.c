#include "utils/cpu_affinity.h"

#ifdef __linux__
#define _GNU_SOURCE
#include <sched.h>
// #include <stdlib.h>
#include <pthread.h>
#include <stdio.h>
#include <unistd.h>

bool SetCpuAffinity() {
  cpu_set_t* cpusetp;
  size_t size;

  cpusetp = CPU_ALLOC(2);
  if (cpusetp == NULL) {
    perror("CPU_ALLOC");
    return false;
  }
  size = CPU_ALLOC_SIZE(2);
  CPU_ZERO_S(size, cpusetp);

  CPU_SET_S(2, size, cpusetp);
  // CPU_SET_S(1, size, cpusetp);
  printf("CPU_COUNT() of set:    %d\n", CPU_COUNT_S(size, cpusetp));

  pthread_t current_thread = pthread_self();
  pthread_setaffinity_np(current_thread, size, cpusetp);

  // sched_setaffinity(0, size, cpusetp);
  return true;
}

bool StickThisThreadToCore(int core_id) {
  int num_cores = sysconf(_SC_NPROCESSORS_ONLN);
  if (core_id < 0 || core_id >= num_cores) {
    printf("invalid core_id\n");
    return false;
  }

  size_t size;
  cpu_set_t* cpuset;

  cpuset = CPU_ALLOC(1);
  size = CPU_ALLOC_SIZE(1);

  CPU_ZERO_S(size, cpuset);
  CPU_SET_S(core_id, size, cpuset);

  pthread_t current_thread = pthread_self();
  if (pthread_setaffinity_np(current_thread, size, cpuset) != 0) return false;

  return true;
}

bool StickThisThreadToCores(int* cores_id, int size) {
  int num_cores = sysconf(_SC_NPROCESSORS_ONLN);

  size_t cpusize;
  cpu_set_t cpuset;

  // cpuset = CPU_ALLOC(1);
  cpusize = CPU_ALLOC_SIZE(size);

  CPU_ZERO_S(cpusize, &cpuset);
  for (int i = 0; i < size; ++i) {
    if (cores_id[i] < 0 || cores_id[i] >= num_cores) {
      printf("invalid core_id %d\n", cores_id[i]);
      continue;
    }
    CPU_SET_S(cores_id[i], cpusize, &cpuset);
  }

  pthread_t current_thread = pthread_self();
  if (pthread_setaffinity_np(current_thread, cpusize, &cpuset) != 0)
    return false;

  return true;
}

#endif  // __linux__
