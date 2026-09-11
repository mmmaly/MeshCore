#pragma once
#include <stdint.h>
#include <string.h>

/*
 * Which spreading factor to talk to a neighbour on, for a radio that receives
 * several SFs at once (LR2021 side detectors) but transmits one at a time.
 *
 * Every received frame is attributed to the node that *transmitted* it: the
 * last hash of a flood path, or, for a zero-hop packet, the sender (an advert's
 * public key, otherwise the source hash). Hashes are stored with the width they
 * came in: two bytes when the path uses 2-byte hashes or the key came from an
 * advert, one byte otherwise. Outgoing frames are matched against the node that
 * will receive them next: the first hop of a direct route, else the destination
 * hash. A 1-byte lookup matches 2-byte entries by their first byte; the most
 * recently heard match wins.
 *
 * Rules (returned SF is 0 = "use the primary"):
 *  - ACKs carry no address: sent on the SF of the last side-SF reception if
 *    that was within reply_window_ms.
 *  - Addressed frames (direct next hop / dest hash) go out on the SF that node
 *    was last heard on, if that was a side SF and within ttl_ms. Hearing a node
 *    on the primary again clears its entry.
 *  - Everything else (floods without a known SF, adverts, group, control): primary.
 *
 * Wire layout (Packet::readFrom): header, [4 transport bytes for route types 0
 * and 3], path_len (bits 6-7 = hash size - 1, bits 0-5 = hop count), path,
 * payload. REQ/RESPONSE/TXT_MSG/PATH payloads start with dest_hash, src_hash;
 * ADVERT with the sender's public key; ANON_REQ with dest_hash then the key.
 * Header-only, no allocation; shared by the Linux host node and the Zephyr port.
 */
class MultiSfMemory {
public:
  uint32_t reply_window_ms = 3000;
  uint32_t ttl_ms = 30u * 60u * 1000u;

  // A frame arrived on `sf` (the primary or a side SF).
  void noteRx(const uint8_t* pkt, int len, uint8_t sf, uint8_t primary, uint32_t now) {
    if (sf && sf != primary) { _last_side_sf = sf; _last_side_ms = now; _have_side = true; }
    Parsed p;
    if (!parse(pkt, len, p)) return;
    uint16_t id; uint8_t idlen;
    if (p.hops > 0) {                                   // transmitted by the last relay
      const uint8_t* h = pkt + p.path_off + (p.hops - 1) * p.hash_size;
      if (p.hash_size == 2) { id = (uint16_t)((h[0] << 8) | h[1]); idlen = 2; }
      else { id = h[0]; idlen = 1; }
    } else if (!senderOf(pkt, len, p, id, idlen)) {
      return;
    }
    upsert(id, idlen, (sf && sf != primary) ? sf : 0, now);
  }

  // SF to transmit `pkt` on (0 = primary). `why` names the rule that fired.
  uint8_t pickTx(const uint8_t* pkt, int len, uint8_t primary, uint32_t now, const char** why) {
    if (why) *why = "";
    Parsed p;
    if (!parse(pkt, len, p)) return 0;
    if (p.type == 0x03) {                               // ACK
      if (_have_side && now - _last_side_ms < reply_window_ms) { if (why) *why = "ack after side-SF rx"; return _last_side_sf; }
      return 0;
    }
    uint16_t id; uint8_t idlen;
    if (p.route == 2 && p.hops > 0) {                   // DIRECT: next hop = first path hash
      const uint8_t* h = pkt + p.path_off;
      if (p.hash_size == 2) { id = (uint16_t)((h[0] << 8) | h[1]); idlen = 2; }
      else { id = h[0]; idlen = 1; }
      if (why) *why = "next hop last heard on this SF";
    } else if (p.type == 0x00 || p.type == 0x01 || p.type == 0x02 || p.type == 0x07 || p.type == 0x08) {
      if (p.payload_off >= len) return 0;
      id = pkt[p.payload_off]; idlen = 1;              // dest hash
      if (why) *why = "destination last heard on this SF";
    } else {
      return 0;
    }
    const Entry* e = find(id, idlen, now);
    if (e && e->sf && e->sf != primary) return e->sf;
    if (why) *why = "";
    return 0;
  }

  // For diagnostics: the SF a node id is remembered on (0 = none/primary).
  uint8_t lookup(uint16_t id, uint8_t idlen, uint32_t now) const {
    const Entry* e = find(id, idlen, now);
    return e ? e->sf : 0;
  }

private:
  enum { MAX_ENTRIES = 32 };
  struct Entry { uint16_t id; uint8_t idlen; uint8_t sf; uint32_t ms; bool used; };
  struct Parsed { uint8_t route, type, hash_size, hops; int path_off, payload_off; };

  Entry _e[MAX_ENTRIES] = {};
  uint8_t _last_side_sf = 0;
  uint32_t _last_side_ms = 0;
  bool _have_side = false;

  static bool parse(const uint8_t* b, int len, Parsed& p) {
    if (len < 3) return false;
    p.route = b[0] & 0x03;
    p.type = (b[0] >> 2) & 0x0F;
    int i = 1 + ((p.route == 0 || p.route == 3) ? 4 : 0);
    if (i >= len) return false;
    uint8_t pl = b[i++];
    p.hash_size = (uint8_t)((pl >> 6) + 1);
    p.hops = (uint8_t)(pl & 63);
    p.path_off = i;
    i += p.hops * p.hash_size;
    if (i >= len) return false;
    p.payload_off = i;
    return true;
  }
  static bool senderOf(const uint8_t* b, int len, const Parsed& p, uint16_t& id, uint8_t& idlen) {
    int o = p.payload_off;
    switch (p.type) {
      case 0x00: case 0x01: case 0x02: case 0x08:       // dest, src
        if (o + 1 >= len) return false;
        id = b[o + 1]; idlen = 1; return true;
      case 0x04:                                        // advert: public key
        if (o + 1 >= len) return false;
        id = (uint16_t)((b[o] << 8) | b[o + 1]); idlen = 2; return true;
      case 0x07:                                        // anon req: dest, public key
        if (o + 2 >= len) return false;
        id = (uint16_t)((b[o + 1] << 8) | b[o + 2]); idlen = 2; return true;
      default: return false;
    }
  }
  static bool matches(const Entry& e, uint16_t id, uint8_t idlen) {
    if (e.idlen == 2 && idlen == 2) return e.id == id;
    if (e.idlen == 1 && idlen == 1) return e.id == id;
    if (e.idlen == 2 && idlen == 1) return (e.id >> 8) == id;
    return (uint16_t)(id >> 8) == e.id;                 // 1-byte entry, 2-byte lookup
  }
  const Entry* find(uint16_t id, uint8_t idlen, uint32_t now) const {
    const Entry* best = nullptr;
    for (const Entry& e : _e) {
      if (!e.used || now - e.ms >= ttl_ms) continue;
      if (!matches(e, id, idlen)) continue;
      if (!best || (int32_t)(e.ms - best->ms) > 0) best = &e;
    }
    return best;
  }
  void upsert(uint16_t id, uint8_t idlen, uint8_t sf, uint32_t now) {
    Entry* slot = nullptr;
    for (Entry& e : _e) {                               // exact key first
      if (e.used && e.idlen == idlen && e.id == id) { slot = &e; break; }
    }
    if (!slot) {
      for (Entry& e : _e) {                             // a 1-byte entry upgraded by a 2-byte sighting
        if (e.used && e.idlen == 1 && idlen == 2 && e.id == (id >> 8)) { slot = &e; break; }
      }
    }
    if (!slot) {
      for (Entry& e : _e) if (!e.used || now - e.ms >= ttl_ms) { slot = &e; break; }
    }
    if (!slot) {                                        // full: evict the oldest
      slot = &_e[0];
      for (Entry& e : _e) if ((int32_t)(e.ms - slot->ms) < 0) slot = &e;
    }
    slot->used = true; slot->id = id; slot->idlen = idlen; slot->sf = sf; slot->ms = now;
  }
};
