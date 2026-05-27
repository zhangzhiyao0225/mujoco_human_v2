cmake_minimum_required(VERSION 3.14 FATAL_ERROR)
project(bitbot_kernel)

if (NOT DEFINED BITBOT_KERNEL_MASTER_PROJECT)
  set(BITBOT_KERNEL_MASTER_PROJECT OFF)
  if (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    set(BITBOT_KERNEL_MASTER_PROJECT ON)
    message(STATUS "CMake version: ${CMAKE_VERSION}")
  endif ()
endif ()

SET(CMAKE_BUILD_TYPE Debug)

if(BITBOT_KERNEL_MASTER_PROJECT)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -g -O0")
  if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
  endif()
endif()

set(BITBOT_KERNEL_INSTALL ${BITBOT_KERNEL_MASTER_PROJECT})

option(BITBOT_DEPENDENCY_USE_PROXY OFF)
option(BITBOT_DEPENDENCY_USE_LOCAL_FILE OFF)

if(BITBOT_DEPENDENCY_USE_LOCAL_FILE)
  if(NOT DEFINED BITBOT_DEPENDENCY_LOCAL_FILE_PATH)
    set(BITBOT_DEPENDENCY_LOCAL_FILE_PATH ${CMAKE_SOURCE_DIR}/dependencies/)
  endif()
endif()

set(BITBOT_KERNEL_INCLUDED TRUE)
set(BITBOT_KERNEL_INCLUDED ${BITBOT_KERNEL_INCLUDED} PARENT_SCOPE)

# include(cmake/dependencies.cmake)
# OPTION(BACKEND_COMM_WS "comm with ws" ON)
# MARK_AS_ADVANCED(BACKEND_COMM_WS)

# IF (BACKEND_COMM_WS)
# MESSAGE(STATUS "KERNEL 1: ${BACKEND_COMM_WS} and val: " $<$<BOOL:${BACKEND_COMM_WS}>:GOD>)
# ELSE()
# MESSAGE(STATUS "KERNEL 0: ${BACKEND_COMM_WS} and val: " $<$<BOOL:${BACKEND_COMM_WS}>:GOD>)
# ENDIF()

aux_source_directory(src/kernel SRC_KERNEL)
aux_source_directory(src/utils SRC_UTILS)
aux_source_directory(src/parser SRC_PARSER)

add_library(BitbotKernel STATIC
            ${SRC_KERNEL}
            ${SRC_UTILS}
)
target_include_directories(BitbotKernel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${BITBOT_KERNEL_INCLUDE_DIR}
)
target_compile_features(BitbotKernel PRIVATE cxx_std_20)  # Require C99 and C++17
target_compile_definitions(BitbotKernel PRIVATE
                          #  LIBUS_USE_OPENSSL
                           $<$<PLATFORM_ID:Windows>:_USE_MATH_DEFINES>
)

target_compile_definitions(BitbotKernel PRIVATE
$<$<BOOL:${BACKEND_COMM_WS}>:BACKEND_COMM_WS>)

MESSAGE(STATUS "BitbotKernel: ${BACKEND_COMM_WS}")
target_link_libraries(BitbotKernel PUBLIC
                      $<$<NOT:$<PLATFORM_ID:Windows>>:m>
                      spdlog::spdlog_header_only
                      pugixml
                      readerwriterqueue
                      concurrentqueue
                      glaze::glaze
)

IF (BACKEND_COMM_WS)
target_link_libraries(BitbotKernel PRIVATE
                      usockets
                      uwebsockets
)
ENDIF()

# target_compile_options(BitbotKernel PRIVATE
#   $<$<CXX_COMPILER_ID:MSVC>:/W4>
#   $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>
# )
