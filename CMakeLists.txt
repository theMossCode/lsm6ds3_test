# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/modules)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lsm6ds3_test)

target_sources(app PRIVATE 
    src/main.c
    src/imu.c
)
