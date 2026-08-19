#pragma once
#include <MeshCore.h>
#include <Utils.h>
#include <cstdio>
#include <ctime>
#include <cmath>
#include <unistd.h>

// The handful of platform services MeshCore expects from a board. On a host
// these are either trivial or meaningless, so they answer honestly rather
// than inventing plausible-looking hardware readings.
class LinuxBoard : public mesh::MainBoard {
public:
  void begin() { _startup = 1; }
  uint16_t getBattMilliVolts() override { return 0; }        // mains powered
  float getMCUTemperature() override { return NAN; }         // no sensor
  const char* getManufacturerName() const override {
    return "MeshCore SDR (RTL-SDR rx, HackRF tx)";
  }
  void reboot() override { _exit(0); }                       // systemd restarts us
  uint8_t getStartupReason() const override { return _startup; }
private:
  uint8_t _startup = 1;
};

class LinuxRNG : public mesh::RNG {
public:
  void random(uint8_t* dest, size_t sz) override {
    FILE* f = fopen("/dev/urandom", "rb");
    if (f) { size_t n = fread(dest, 1, sz, f); (void)n; fclose(f); }
    else for (size_t i = 0; i < sz; i++) dest[i] = (uint8_t)rand();
  }
};

class LinuxRTCClock : public mesh::RTCClock {
public:
  uint32_t getCurrentTime() override { return (uint32_t)time(NULL); }
  void setCurrentTime(uint32_t time) override { /* host clock is NTP-disciplined */ }
};
