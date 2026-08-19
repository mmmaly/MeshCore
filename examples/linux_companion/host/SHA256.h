#pragma once
// rweather/Crypto SHA256 API (including its HMAC helpers), over OpenSSL EVP.
// Deliberately avoids <openssl/sha.h>: its SHA256() function name collides
// with this class and makes firmware sources fail to compile.
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <vector>
#include <openssl/evp.h>
#include <openssl/hmac.h>

class SHA256 {
  std::vector<uint8_t> _buf;
public:
  static const size_t HASH_SIZE = 32;
  void reset() { _buf.clear(); }
  void update(const void* data, size_t len) {
    const uint8_t* b = (const uint8_t*)data;
    _buf.insert(_buf.end(), b, b + len);
  }
  void finalize(void* hash, size_t len) {
    uint8_t full[EVP_MAX_MD_SIZE];
    unsigned int n = 0;
    EVP_Digest(_buf.data(), _buf.size(), full, &n, EVP_sha256(), nullptr);
    memcpy(hash, full, len < n ? len : n);
    _buf.clear();
  }
  // rweather pattern: resetHMAC(key) -> update(data)... -> finalizeHMAC(key, out)
  void resetHMAC(const void* key, size_t keyLen) { _buf.clear(); }
  void finalizeHMAC(const void* key, size_t keyLen, void* hash, size_t hashLen) {
    unsigned int n = 0;
    uint8_t mac[EVP_MAX_MD_SIZE];
    HMAC(EVP_sha256(), key, (int)keyLen, _buf.data(), _buf.size(), mac, &n);
    memcpy(hash, mac, hashLen < n ? hashLen : n);
    _buf.clear();
  }
};
