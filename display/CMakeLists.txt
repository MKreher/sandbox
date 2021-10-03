# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

set(SHIELD waveshare_epaper_gdeh0154a07)
#set(SHIELD ssd1306_128x64)
#set(SHIELD st7789v_tl019fqv01)
#set(SHIELD st7735r_ada_160x128)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sandbox_display)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
