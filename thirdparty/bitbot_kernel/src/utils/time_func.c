#include "utils/time_func.h"

#include <math.h>

#ifdef __linux__
#include <bits/types/clockid_t.h>
#include <linux/time.h>

extern int clock_nanosleep(clockid_t clockid, int flags,
                           const struct timespec* request,
                           struct timespec* _Nullable_remain);

struct timespec DiffTime(struct timespec start, struct timespec end) {
  struct timespec temp;
  if ((end.tv_nsec - start.tv_nsec) < 0) {
    temp.tv_sec = end.tv_sec - start.tv_sec - 1;
    temp.tv_nsec = 1000000000 - start.tv_nsec + end.tv_nsec;
  } else {
    temp.tv_sec = end.tv_sec - start.tv_sec;
    temp.tv_nsec = end.tv_nsec - start.tv_nsec;
  }
  return temp;
}

inline void SleepS(uint32_t s) {
  struct timespec deadline;

  deadline.tv_sec = s;
  deadline.tv_nsec = 0;

  clock_nanosleep(CLOCK_MONOTONIC, 0, &deadline, NULL);
}

inline void SleepMS(uint32_t ms) {
  struct timespec deadline;

  deadline.tv_sec = ms / 1000;
  deadline.tv_nsec = (ms % 1000) * 1000000;

  clock_nanosleep(CLOCK_MONOTONIC, 0, &deadline, NULL);
}

inline void SleepUS(uint32_t us) {
  struct timespec deadline;

  deadline.tv_sec = us / 1000000;
  deadline.tv_nsec = (us % 1000000) * 1000;

  clock_nanosleep(CLOCK_MONOTONIC, 0, &deadline, NULL);
}

inline void SleepNS(uint32_t ns) {
  struct timespec deadline;

  deadline.tv_sec = ns / 1000000000;
  deadline.tv_nsec = ns % 1000000000;

  clock_nanosleep(CLOCK_MONOTONIC, 0, &deadline, NULL);
}

#elif _WIN32

#include <Windows.h>

void SleepS(uint32_t s) { Sleep(s * 1000); }

void SleepMS(uint32_t ms) { Sleep(ms); }

#endif  // __linux__
