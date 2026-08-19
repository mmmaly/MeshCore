#pragma once
// rweather/Crypto AES128 API, backed by OpenSSL. Semantics are standard
// AES-128 ECB single-block, so this is exact, not an approximation.
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <openssl/evp.h>

class AES128 {
  uint8_t _key[16];
public:
  AES128() { memset(_key, 0, sizeof(_key)); }
  void setKey(const uint8_t* key, size_t keySize) {
    memcpy(_key, key, keySize < 16 ? keySize : 16);
  }
  void encryptBlock(uint8_t* output, const uint8_t* input) { crypt(output, input, 1); }
  void decryptBlock(uint8_t* output, const uint8_t* input) { crypt(output, input, 0); }
private:
  void crypt(uint8_t* out, const uint8_t* in, int enc) {
    EVP_CIPHER_CTX* ctx = EVP_CIPHER_CTX_new();
    if (!ctx) return;
    EVP_CipherInit_ex(ctx, EVP_aes_128_ecb(), nullptr, _key, nullptr, enc);
    EVP_CIPHER_CTX_set_padding(ctx, 0);
    int len = 0;
    EVP_CipherUpdate(ctx, out, &len, in, 16);
    EVP_CIPHER_CTX_free(ctx);
  }
};
