# lr2021_serial: a LoRa modem on the Seeed XIAO nRF54L15 + Wio-LR2021
# (Semtech LR2021 EVK) that a Linux host drives over the board's console UART.
# RadioLib's LR2021 driver in generic (non-Arduino) mode does all the radio
# work; see README.md for the line protocol and SerialRadio.cpp for the
# consumer.
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lr2021_serial)

# RadioLib checkout (the commit MeshCore's platformio.ini pins is known good
# for the LR2021; anything >= 7.7 with setSideDetector() should do).
if(DEFINED ENV{RADIOLIB_DIR})
  set(RADIOLIB_DIR $ENV{RADIOLIB_DIR})
else()
  set(RADIOLIB_DIR $ENV{HOME}/RadioLib)
endif()
set(RADIOLIB_SRC ${RADIOLIB_DIR}/src)
if(NOT EXISTS ${RADIOLIB_SRC}/RadioLib.h)
  message(FATAL_ERROR "RadioLib not found at ${RADIOLIB_DIR} (set RADIOLIB_DIR)")
endif()

file(GLOB_RECURSE RL_SOURCES ${RADIOLIB_SRC}/*.cpp)
list(FILTER RL_SOURCES EXCLUDE REGEX "/hal/")   # generic HAL only; ours is in src/

target_sources(app PRIVATE src/main.cpp ${RL_SOURCES})
target_include_directories(app PRIVATE src ${RADIOLIB_SRC})
# GODMODE exposes setLoRaPacketParams(), needed to re-assert the RX max
# length after a transmit (the chip keeps the last TX length otherwise and
# clips longer incoming packets - found during MeshCore's LR2021 bring-up).
target_compile_definitions(app PRIVATE RADIOLIB_GODMODE=1 RADIOLIB_LOW_LEVEL=1)
target_compile_options(app PRIVATE -Wno-unused-parameter)
