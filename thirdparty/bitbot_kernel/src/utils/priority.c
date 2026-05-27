#include "utils/priority.h"

#include <string.h>

#ifdef __linux__
#include <linux/sched.h>
#include <linux/sched/types.h>
#include <stdio.h>
#include <sys/syscall.h>
#include <sys/types.h>

extern long syscall(long number, ...);

long sched_setattr(pid_t pid, const struct sched_attr* attr,
                   unsigned int flags) {
  return syscall(__NR_sched_setattr, pid, attr, flags);
}

bool setProcessHighPriority(unsigned int priority) {
  /* Get high priority */
  struct sched_attr attr;
  memset(&attr, 0, sizeof(attr));
  attr.size = sizeof(attr);
  attr.sched_policy = SCHED_RR;
  attr.sched_priority = priority;
  if (sched_setattr(0, &attr, 0) < 0) {
    printf("sched_setattr failed\n");
    return false;
  }
  return true;
}

bool SetDeadlinePolicy(unsigned int runtime, unsigned int period,
                       unsigned int deadline) {
  /* Get high priority */
  struct sched_attr attr;
  memset(&attr, 0, sizeof(attr));
  attr.size = sizeof(attr);
  attr.sched_policy = SCHED_DEADLINE;
  attr.sched_runtime = runtime;
  attr.sched_period = period;
  attr.sched_deadline = deadline;
  if (sched_setattr(0, &attr, 0) < 0) {
    printf("sched_setattr failed\n");
    return false;
  }
  return true;
}
#endif  // __linux__
