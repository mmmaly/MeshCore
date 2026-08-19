#pragma once
// Minimal Arduino surface for a host build. Unlike test/mocks/Arduino.h this
// uses the real monotonic clock - a running node needs true time, not a
// counter that only advances when delay() is called.
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <ctime>
#include <unistd.h>
#include "Stream.h"

using std::isnan;

inline uint32_t millis() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint32_t)(ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL);
}
inline uint32_t micros() {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint32_t)(ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL);
}
inline void delay(uint32_t ms) { usleep(ms * 1000); }
inline void delayMicroseconds(uint32_t us) { usleep(us); }
inline long random(long max) { return max > 0 ? ::random() % max : 0; }
inline long random(long min, long max) { return max > min ? min + ::random() % (max - min) : min; }

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif
