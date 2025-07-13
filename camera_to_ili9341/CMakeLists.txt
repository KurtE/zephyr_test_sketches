# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ili9341_touch)

#include(../common/common.cmake)
FILE(GLOB app_sources src/*.cpp  app_sources src/*.c app_sources ../common/src/*.cpp ../ILI9341_library/src/*.cpp ../ILI9341_library/src/*.c ../ST77XX_library/src/*.cpp ../ST77XX_library/src/*.c)
target_include_directories(app PRIVATE ../common/include ../ILI9341_library/include ../ST77XX_library/include)
target_sources(app PRIVATE ${app_sources})
