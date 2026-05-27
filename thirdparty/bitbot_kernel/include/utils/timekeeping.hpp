#pragma once

#include "bitbot_kernel/utils/logger.h"

#include <iostream>
#include <chrono>
#include <functional>

namespace bitbot
{
  template<typename Callable, typename ...Args>
  void TimeKeeping(Callable&& func, Args&&... args)
  {
    using namespace std::chrono;

    auto start = high_resolution_clock::now(); // 记录开始时间

    std::invoke(std::forward<Callable>(func), std::forward<Args>(args)...); // 调用传入的函数

    auto stop = high_resolution_clock::now(); // 记录结束时间

    auto duration = duration_cast<microseconds>(stop - start); // 计算运行时间

    Logger().ConsoleLogger()->debug("函数运行时间: {}us", duration.count());
  }
}