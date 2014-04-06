#
# Copyright (c) 2014 Courierdrone, courierdrone.com
# Distributed under the MIT License, available in the file LICENSE.
# Author: Pavel Kirienko <pavel.kirienko@courierdrone.com>
#

cmake_minimum_required(VERSION 2.8)

project(crdr_gps_test)

find_library(UAVCAN_LIB uavcan REQUIRED)

set(CMAKE_CXX_FLAGS "-Wall -Wextra -Werror -pedantic -std=c++0x -g3")  # GCC or Clang

add_executable(subscriber subscriber.cpp)
target_link_libraries(subscriber ${UAVCAN_LIB} rt)
