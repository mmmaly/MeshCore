#pragma once
// The densaugeo/base64 Arduino API used by BaseChatMesh::addChannel.
#include <stdint.h>
#include <stddef.h>
#include <string.h>

inline int decode_base64_length(const unsigned char* in, size_t len) {
  size_t pad = 0;
  if (len >= 1 && in[len - 1] == '=') pad++;
  if (len >= 2 && in[len - 2] == '=') pad++;
  return (int)(len / 4 * 3 - pad);
}
inline int decode_base64(const unsigned char* in, size_t inLen, unsigned char* out) {
  static const char* T = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  int val = 0, valb = -8, n = 0;
  for (size_t i = 0; i < inLen; i++) {
    const char* p = strchr(T, in[i]);
    if (!p) { if (in[i] == '=') break; else continue; }
    val = (val << 6) + (int)(p - T);
    valb += 6;
    if (valb >= 0) { out[n++] = (unsigned char)((val >> valb) & 0xFF); valb -= 8; }
  }
  return n;
}
inline int decode_base64(const unsigned char* in, unsigned char* out) {
  return decode_base64(in, strlen((const char*)in), out);
}
inline int encode_base64_length(size_t len) { return (int)((len + 2) / 3 * 4); }
inline int encode_base64(const unsigned char* in, size_t inLen, unsigned char* out) {
  static const char* T = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  int val = 0, valb = -6, n = 0;
  for (size_t i = 0; i < inLen; i++) {
    val = (val << 8) + in[i]; valb += 8;
    while (valb >= 0) { out[n++] = (unsigned char)T[(val >> valb) & 0x3F]; valb -= 6; }
  }
  if (valb > -6) out[n++] = (unsigned char)T[((val << 8) >> (valb + 8)) & 0x3F];
  while (n % 4) out[n++] = '=';
  out[n] = 0;
  return n;
}
