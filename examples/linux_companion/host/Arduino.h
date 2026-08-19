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
inline long random(long hi) { return hi > 0 ? ::random() % hi : 0; }
inline long random(long lo, long hi) { return hi > lo ? lo + ::random() % (hi - lo) : lo; }

// Arduino's global Serial. MeshCore's debug macros expand to Serial.printf,
// and firmware sources print to it directly; on a host it goes to stderr so
// it interleaves with the daemon's own logging.
#include <cstdarg>
class HostSerial : public Stream {
public:
  void begin(unsigned long) {}
  operator bool() const { return true; }
  int printf(const char* fmt, ...) {
    va_list ap; va_start(ap, fmt);
    int n = vfprintf(stderr, fmt, ap);
    va_end(ap); return n;
  }
  using Stream::print;      // do not hide the inherited print/println overloads
  using Stream::println;

  // Stream's virtuals: everything else (print/println) is inherited
  size_t write(uint8_t b) override { return fputc(b, stderr) >= 0 ? 1 : 0; }
  size_t write(const uint8_t* buf, size_t size) override {
    return fwrite(buf, 1, size, stderr);
  }
  size_t print(int v, int r = DEC) override { return fprintf(stderr, "%d", v); }
  size_t print(unsigned int v, int r = DEC) override { return fprintf(stderr, "%u", v); }
  size_t print(long v, int r = DEC) override { return fprintf(stderr, "%ld", v); }
  size_t print(unsigned long v, int r = DEC) override { return fprintf(stderr, "%lu", v); }
  size_t print(double v, int p = 2) override { return fprintf(stderr, "%f", v); }
  int available() override { return 0; }
  int read() override { return -1; }
  void flush() override { fflush(stderr); }
};
inline HostSerial Serial;

// AVR-isms glibc/macOS lack
inline char* itoa(int v, char* s, int base) {
  sprintf(s, base == 16 ? "%x" : "%d", v); return s;
}
inline char* ltoa(long v, char* s, int base) {
  sprintf(s, base == 16 ? "%lx" : "%ld", v); return s;
}
inline char* ultoa(unsigned long v, char* s, int base) {
  sprintf(s, base == 16 ? "%lx" : "%lu", v); return s;
}
inline char* dtostrf(double v, int width, int prec, char* s) {
  sprintf(s, "%*.*f", width, prec, v); return s;
}

#ifndef constrain
#define constrain(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
#endif
inline void randomSeed(unsigned long seed) { srandom((unsigned int)seed); }

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

// Arduino's min/max are macros, so they must come after every declaration
// that uses those words as identifiers (random(lo, hi) above, for one).
#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif
