/* Import an existing MeshCore node identity into a linux_companion data dir.
 *
 * Input is the 32-byte Ed25519 seed (64 hex chars; a 128-hex key of
 * seed||public is accepted and the seed taken from the front). The key is
 * expanded with the same vendored orlp implementation the firmware uses, so
 * the result is byte-identical to what the node would have generated itself.
 *
 *   ./import_identity <seed-hex> <data-dir>
 *
 * Writes <data-dir>/identity/_main.id in the firmware's format: the 32-byte
 * public key followed by the 64-byte private key.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#define ED25519_NO_SEED 1
#include <ed_25519.h>

static int unhex(const char* s, unsigned char* out, int n) {
  for (int i = 0; i < n; i++) {
    unsigned v;
    if (sscanf(s + i * 2, "%2x", &v) != 1) return 0;
    out[i] = (unsigned char)v;
  }
  return 1;
}

int main(int argc, char** argv) {
  if (argc < 3) {
    fprintf(stderr, "usage: %s <seed-hex-64-or-128> <data-dir>\n", argv[0]);
    return 1;
  }
  size_t len = strlen(argv[1]);
  if (len != 64 && len != 128) {
    fprintf(stderr, "key must be 64 hex chars (seed) or 128 (seed||public)\n");
    return 1;
  }
  unsigned char seed[32], pub[32], prv[64];
  if (!unhex(argv[1], seed, 32)) { fprintf(stderr, "bad hex\n"); return 1; }

  ed25519_create_keypair(pub, prv, seed);

  if (len == 128) {   /* verify against the public half we were given */
    unsigned char expect[32];
    if (unhex(argv[1] + 64, expect, 32) && memcmp(expect, pub, 32) != 0) {
      fprintf(stderr, "refusing: derived public key does not match the one supplied\n");
      return 1;
    }
  }

  char dir[512], path[600];
  snprintf(dir, sizeof(dir), "%s/identity", argv[2]);
  mkdir(argv[2], 0700);
  mkdir(dir, 0700);
  snprintf(path, sizeof(path), "%s/_main.id", dir);

  FILE* f = fopen(path, "wb");
  if (!f) { perror("open"); return 1; }
  fwrite(pub, 1, 32, f);
  fwrite(prv, 1, 64, f);
  fclose(f);
  chmod(path, 0600);

  printf("wrote %s\n  public key: ", path);
  for (int i = 0; i < 32; i++) printf("%02X", pub[i]);
  printf("\n");
  return 0;
}
