# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(usb_to_serial)

#include(../common/common.cmake)
FILE(GLOB app_sources src/*.cpp  app_sources src/*.c app_sources ../common/src/*.cpp)
target_include_directories(app PRIVATE ../common/include )
target_sources(app PRIVATE ${app_sources})
