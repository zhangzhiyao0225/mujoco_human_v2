#pragma once                                                                       

// for cmake
// 用于在CMakeLists文件中解析用
// 0.1.0                                                                 
#define BITBOT_KERNEL_VER_MAJOR 0                                                      
#define BITBOT_KERNEL_VER_MINOR 1                                                      
#define BITBOT_KERNEL_VER_PATCH 0                                                      

#define BITBOT_KERNEL_VERSION (BITBOT_KERNEL_VER_MAJOR * 10000 + BITBOT_KERNEL_VER_MINOR * 100 + BITBOT_KERNEL_VER_PATCH)

// for source code
// 用于在项目源码中获取版本号字符串
// v0.1.0                                                           
#define _BITBOT_KERNEL_STR(s) #s                                                       
#define HELLO_PROJECT_VERSION(major, minor, patch) "v" _BITBOT_KERNEL_STR(major.minor.patch)
