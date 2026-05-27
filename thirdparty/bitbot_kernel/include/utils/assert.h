#pragma once

#include <iostream>

#ifndef NDEBUG
#   define bitbot_assert(condition, message) \
    do { \
        if (! (condition)) { \
            std::cerr << "Assertion `" #condition "` failed in " << __FILE__ \
                      << " line " << __LINE__ << ": " << message << std::endl; \
            std::terminate(); \
        } \
    } while (false)
#else
#   define bitbot_assert(condition, message) \
    do { \
        if (! (condition)) { \
            std::cerr << "Error. " << message << std::endl; \
            std::terminate(); \
        } \
    } while (false)
#endif