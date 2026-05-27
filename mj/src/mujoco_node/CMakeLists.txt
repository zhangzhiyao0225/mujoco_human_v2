cmake_minimum_required(VERSION 3.8)
project(mujoco_node)

# 设置 C++20 标准
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 明确指定使用 C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

list(APPEND CMAKE_PREFIX_PATH "src/mujoco_node/cmake")
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(mujoco REQUIRED)

# ===== MuJoCo 手动配置开始 =====
# 优先从环境变量 MUJOCO_ROOT 里拿路径
if(NOT DEFINED ENV{MUJOCO_ROOT})
  message(FATAL_ERROR "环境变量 MUJOCO_ROOT 未设置，请先在 ~/.bashrc 里 export MUJOCO_ROOT=/home/huahui/zzy/mujoco-3.2.7")
endif()

set(MUJOCO_ROOT $ENV{MUJOCO_ROOT})

# 找头文件：mujoco/mujoco.h
find_path(MUJOCO_INCLUDE_DIR mujoco/mujoco.h
  HINTS
    "${MUJOCO_ROOT}/include"
    "${MUJOCO_ROOT}"
)

# 找动态库：libmujoco.so
find_library(MUJOCO_LIBRARY mujoco
  HINTS
    "${MUJOCO_ROOT}/build/lib"
    "${MUJOCO_ROOT}/lib"
    "${MUJOCO_ROOT}/bin"
    "${MUJOCO_ROOT}"
)

if(NOT MUJOCO_INCLUDE_DIR OR NOT MUJOCO_LIBRARY)
  message(FATAL_ERROR "MuJoCo not found. 请检查 MUJOCO_ROOT 路径是否正确：${MUJOCO_ROOT}")
endif()

message(STATUS "Found MuJoCo include: ${MUJOCO_INCLUDE_DIR}")
message(STATUS "Found MuJoCo library: ${MUJOCO_LIBRARY}")

include_directories(${MUJOCO_INCLUDE_DIR})
# ===== MuJoCo 手动配置结束 =====


file(GLOB_RECURSE source ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cc)

add_executable(mujoco_node ${source})
target_compile_features(mujoco_node PUBLIC cxx_std_20)  # 确保使用 C++20 特性

ament_target_dependencies(mujoco_node rclcpp custom_msgs sensor_msgs)
# target_link_libraries(mujoco_node mujoco glfw)
find_package(glfw3 REQUIRED)
target_link_libraries(mujoco_node
    ${MUJOCO_LIBRARY}
    glfw
)


target_include_directories(mujoco_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/mujoco_node>
    $<INSTALL_INTERFACE:include/mujoco_node>
)

install(TARGETS mujoco_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
