#pragma once
// rweather/Crypto's Ed25519 class surface, backed by the orlp ed25519 that
// MeshCore already vendors in lib/ed25519 - so signatures verify with the
// exact same implementation the firmware uses on hardware.
#include <stdint.h>
#include <stddef.h>
#define ED25519_NO_SEED 1
#include <ed_25519.h>

class Ed25519 {
public:
  static bool verify(const void* signature, const void* publicKey,
                     const void* message, size_t len) {
    return ed25519_verify((const unsigned char*)signature,
                          (const unsigned char*)message, len,
                          (const unsigned char*)publicKey) != 0;
  }
};
