cmake_minimum_required(VERSION 3.20.0)
project(ovinf)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# If march=native is turned on, you should add the same flag to the root project.
# Otherwise, you may get memory double free error.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

# set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};/home/dknt/Library/openvino-2025.0.0/runtime/cmake")
# set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};/home/dknt/Library/openvino_u24_2025.1.0/runtime/cmake")
# list(APPEND CMAKE_PREFIX_PATH "${OPENVINO_ROOT_DIR}/cmake")

# find_package(OpenVINO REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/utils SRC_UTILS)
MESSAGE(STATUS "111 ${CMAKE_CURRENT_SOURCE_DIR}")
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/ovinf SRC_OVINF)

add_library(ovinf SHARED
  ${SRC_UTILS}
  ${SRC_OVINF}
)
target_link_directories(ovinf PUBLIC ${OPENVINO_LIB_DIR})
target_include_directories(ovinf PUBLIC
  ${OPENVINO_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${yaml-cpp_INCLUDE_DIRS}
)
target_link_libraries(ovinf
  # openvino::runtime
  yaml-cpp
  readerwriterqueue
  openvino
)

set(BUILD_TEST OFF CACHE BOOL "Build test executables")
if (BUILD_TEST) 
  add_subdirectory(test)
endif()

