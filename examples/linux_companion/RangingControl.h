#pragma once
// Private MeshCore control-packet subtype for LR2021 time-of-flight ranging,
// shared by the Linux companion and the XIAO nRF54L15 Zephyr port.
//
// The requester sends a zero-hop CONTROL packet (PAYLOAD_TYPE_CONTROL,
// payload[0] bit 7 set so Mesh routes it to onControlDataRecv), then, after
// RANGING_SETUP_MS, runs `count` RTToF exchanges as manager. The addressed
// node (peer prefix = first 4 bytes of its public key) enters RTToF
// subordinate mode for the window as soon as it receives the request. Both
// use the same band/bandwidth/SF and the same Tx->Rx delay calibration. The
// subordinate's ranging address is the first 4 bytes of its own public key,
// and that is the address the requester asks for.
//
// App side (private companion command, intercepted by the serial interface):
//   CMD  0xF0: [F0][peer pubkey 32][count u8]
//   PUSH 0xF0: [F0][status u8][valid u8][count u8][median_cm i32][min_cm i32][max_cm i32]
//              status 0 = ok, 1 = no valid exchange, 2 = radio does not support ranging, 3 = busy
#include <stdint.h>
#include <string.h>

#define RANGING_CTRL_SUBTYPE   0xF1     // payload[0] of the control packet
#define RANGING_CMD_CODE       0xF0     // app <-> node private frame code
#define RANGING_SETUP_MS       350      // requester waits this long after its request went out
#define RANGING_PER_EXCH_MS    150      // subordinate window budget per exchange
#define RANGING_WINDOW_SLACK_MS 1500

struct RangingRequest {
  uint8_t  peer[4];       // first 4 bytes of the subordinate's public key
  uint32_t freq_hz;
  uint32_t bw_hz;
  uint8_t  sf;
  uint8_t  count;
  uint32_t delay;         // Tx->Rx delay indicator both sides program (0 = chip table)
};
#define RANGING_REQ_LEN 19

static inline size_t rangingEncodeRequest(const RangingRequest& r, uint8_t* out) {
  out[0] = RANGING_CTRL_SUBTYPE;
  memcpy(out + 1, r.peer, 4);
  uint32_t v = r.freq_hz; memcpy(out + 5, &v, 4);
  v = r.bw_hz; memcpy(out + 9, &v, 4);
  out[13] = r.sf; out[14] = r.count;
  v = r.delay; memcpy(out + 15, &v, 4);
  return RANGING_REQ_LEN;
}
static inline bool rangingDecodeRequest(const uint8_t* in, size_t len, RangingRequest& r) {
  if (len < RANGING_REQ_LEN || in[0] != RANGING_CTRL_SUBTYPE) return false;
  memcpy(r.peer, in + 1, 4);
  memcpy(&r.freq_hz, in + 5, 4); memcpy(&r.bw_hz, in + 9, 4);
  r.sf = in[13]; r.count = in[14];
  memcpy(&r.delay, in + 15, 4);
  return true;
}
static inline uint32_t rangingAddrFromPubKey(const uint8_t* pub) {   // big-endian u32 of the first 4 bytes
  return ((uint32_t)pub[0] << 24) | ((uint32_t)pub[1] << 16) | ((uint32_t)pub[2] << 8) | pub[3];
}
static inline uint32_t rangingWindowMs(uint8_t count) { return (uint32_t)count * RANGING_PER_EXCH_MS + RANGING_WINDOW_SLACK_MS; }

struct RangingResult {
  uint8_t status = 2, valid = 0, count = 0;
  int32_t median_cm = 0, min_cm = 0, max_cm = 0;
};
static inline size_t rangingEncodeResult(const RangingResult& r, uint8_t* out) {
  out[0] = RANGING_CMD_CODE; out[1] = r.status; out[2] = r.valid; out[3] = r.count;
  memcpy(out + 4, &r.median_cm, 4); memcpy(out + 8, &r.min_cm, 4); memcpy(out + 12, &r.max_cm, 4);
  return 16;
}
