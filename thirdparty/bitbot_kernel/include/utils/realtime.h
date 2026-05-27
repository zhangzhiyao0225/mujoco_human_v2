#pragma once

#include "utils/logger.h"

#ifdef __linux__

#include <fcntl.h>
#include <stdint.h>
#include <sys/mman.h>
#include <unistd.h>

namespace bitbot {
void RtAppStart();
void RtAppEnd();
}  // namespace bitbot

#endif  // __linux__
