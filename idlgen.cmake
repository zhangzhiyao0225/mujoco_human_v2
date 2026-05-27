cmake_minimum_required(VERSION 3.20)
project(bitbot_efc)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

set(IDL_COMPILER ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Fast-DDS-Gen/scripts/fastddsgen)
MESSAGE(STATUS "IDL_COMPILER: ${IDL_COMPILER}")

# Compile DDS idl files
find_package(fastcdr REQUIRED)
find_package(fastrtps REQUIRED)

# Generate DDS lib for each message in idl directory
set(EFC_DDS_SOURCES "")
set(EFC_DDS_INCLUDES "")

set(IDL_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/idl")
set(IDL_GEN_DIR "${IDL_ROOT_DIR}/gen")
set(IDL_SRC_DIR "${IDL_ROOT_DIR}/src")
set(IDL_GEN_INC_DIR "${IDL_GEN_DIR}")

file(GLOB IDLs "${IDL_SRC_DIR}/*.idl")
foreach(IDL ${IDLs})
	get_filename_component(IDL_NAME ${IDL} NAME_WE)

  SET(CUR_IDL_SRC_DIR ${IDL_GEN_DIR}/${IDL_NAME})

  message(STATUS "Generating DDS for ${IDL}: Name: ${IDL_NAME}, Dir: ${CUR_IDL_SRC_DIR}")
  file(MAKE_DIRECTORY ${CUR_IDL_SRC_DIR})

  set(IDL_OUT_CPP "${CUR_IDL_SRC_DIR}/${IDL_NAME}.cxx")
  set(IDL_OUT_HPP "${CUR_IDL_SRC_DIR}/${IDL_NAME}.h")
  set(IDL_OUT_PUBSUB_CPP "${CUR_IDL_SRC_DIR}/${IDL_NAME}PubSubTypes.cxx")
  set(IDL_OUT_PUBSUB_HPP "${CUR_IDL_SRC_DIR}/${IDL_NAME}PubSubTypes.h")
  add_custom_command(
        OUTPUT ${IDL_OUT_CPP} ${IDL_OUT_HPP} ${IDL_OUT_PUBSUB_CPP} ${IDL_OUT_PUBSUB_HPP}

        COMMAND ${IDL_COMPILER} -typeros2 -d ${CUR_IDL_SRC_DIR} ${IDL}
                # --cpp_out ${IDL_PATH}

        DEPENDS ${IDL}

        COMMENT "Generating C++ from ${IDL}"

        VERBATIM
    )
  
  Aux_source_directory(${CUR_IDL_SRC_DIR} DDS_SOURCCES_SUBDIR)
  list(APPEND EFC_DDS_SOURCES ${DDS_SOURCCES_SUBDIR})
  list(APPEND EFC_DDS_INCLUDES ${CUR_IDL_SRC_DIR})

    # list(APPEND EFC_DDS_SOURCES ${OUT_CPP})
    # list(APPEND EFC_DDS_INCLUDES ${OUT_HPP})

    # add_custom_command(
	# 	OUTPUT ${IDL_DIR}/${IDL_NAME}PubSubTypes.h
	# 	COMMAND ${IDL_COMPILER} ${IDL}
	# )
endforeach(IDL)
message(STATUS "Generating DDS source ${EFC_DDS_SOURCES}")
message(STATUS "Generating DDS include ${EFC_DDS_INCLUDES}")

# foreach(SUBDIR ${IDLs})
# 	
# endforeach()
add_library(efc_dds_lib
  ${EFC_DDS_SOURCES}
  ${EFC_DDS_INCLUDES}
)
target_link_libraries(efc_dds_lib fastrtps fastcdr)
return()

# Bitbot EFC Library

set(BITBOT_DEPENDENCY_USE_PROXY TRUE)
set(BITBOT_DEPENDENCY_USE_LOCAL_FILE TRUE)
#set(BITBOT_DEPENDENCY_LOCAL_FILE_PATH ${CMAKE_SOURCE_DIR}/dependencies)
#include(dependencies/bitbot.cmake)
#BitbotAddDependency(
#  NAME           bitbot_kernel
#  FILE_NAME      "bitbot_kernel-main.zip"
#  FILE_PATH      ${BITBOT_DEPENDENCY_LOCAL_FILE_PATH}
#  USE_LOCAL_FILE TRUE
#)

find_package(yaml-cpp REQUIRED)
include_directories("/usr/include/eigen3")

SET(BITBOT_SOURCE_DIR ${BITBOT_SRC_ROOT_DIR}/${BITBOT_TYPE})
aux_source_directory(${BITBOT_SOURCE_DIR}/device BITBOT_EFC_DEVICE_SRC)
aux_source_directory(${BITBOT_SOURCE_DIR}/bus BITBOT_EFC_BUS_SRC)
set(BITBOT_EFC_SRC
  ${BITBOT_EFC_DEVICE_SRC}
  ${BITBOT_EFC_BUS_SRC}
  ${BITBOT_EFC_KERNEL_SRC}
  ${BITBOT_CURRENT_FRONTEND_SRC_DIR}
)

add_library(${BITBOT_LIB_NAME} ${BITBOT_EFC_SRC})
target_include_directories(${BITBOT_LIB_NAME} PUBLIC $<BUILD_INTERFACE:${THIRD_PARTY_INCLUDE}> $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
  ${IDL_GEN_INC_DIR}
  ${BITBOT_INCLUDES}
  ${BITBOT_KERNEL_INCLUDE_DIR}
  ${GLAZE_INCLUDE_DIR}
  ${BITBOT_CURRENT_FRONTEND_INC_DIR}
  )

# Bitbot main app
# FIXME: removed by xky
# include(FetchContent)
# FetchContent_Declare(
#   ovinf
#   GIT_REPOSITORY https://github.com/Dknt0/ovinf.git 
#   GIT_TAG main
# )
# FetchContent_MakeAvailable(ovinf)

include(thirdparties.cmake)


list(APPEND CMAKE_PREFIX_PATH "${PROJECT_SOURCE_DIR}/src/mj/src/custom_msgs/install")
list(APPEND CMAKE_PREFIX_PATH "${OPENVINO_ROOT_DIR}/cmake")

find_package(OpenVINO REQUIRED)

add_executable(${BITBOT_APP_NAME} main.cc)

target_include_directories(${BITBOT_APP_NAME} PUBLIC
  include
  ${BITBOT_INCLUDES}
  ${OVINF_INCLUDE_DIR}
  ${OPENVINO_INCLUDE_DIR}
  ${CONCURRENTQUEUE_INCLUDE_DIR}
  ${BITBOT_ROS_INC_DIR}
)

target_link_libraries(${BITBOT_APP_NAME}
  ${BITBOT_LIB_NAME}
  ovinf
  ${BITBOT_ROS_LIBS}
)

# Tests
set(BUILD_TEST ON)
if(BUILD_TEST)
  include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${BITBOT_INCLUDES}
    ${IDL_GEN_INC_DIR}
  )
endif()
