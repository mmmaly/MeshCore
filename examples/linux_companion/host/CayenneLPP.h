#pragma once
// Minimal Cayenne LPP encoder - the subset MeshCore's telemetry uses.
// Format is [channel][type][big-endian value], per the LPP spec.
#include <stdint.h>
#include <stddef.h>
#include <string.h>

class CayenneLPP {
  uint8_t* _buf; size_t _cap, _len;
public:
  explicit CayenneLPP(size_t size) : _cap(size), _len(0) { _buf = new uint8_t[size]; }
  ~CayenneLPP() { delete[] _buf; }
  void reset() { _len = 0; }
  uint8_t* getBuffer() { return _buf; }
  size_t getSize() const { return _len; }

  bool addVoltage(uint8_t channel, float volts) {      // type 116, 0.01 V
    return add(channel, 116, (uint16_t)(volts * 100.0f + 0.5f));
  }
  bool addTemperature(uint8_t channel, float celsius) { // type 103, 0.1 C signed
    return add(channel, 103, (uint16_t)(int16_t)(celsius * 10.0f));
  }
private:
  bool add(uint8_t ch, uint8_t type, uint16_t v) {
    if (_len + 4 > _cap) return false;
    _buf[_len++] = ch; _buf[_len++] = type;
    _buf[_len++] = (uint8_t)(v >> 8); _buf[_len++] = (uint8_t)(v & 0xFF);
    return true;
  }
};
