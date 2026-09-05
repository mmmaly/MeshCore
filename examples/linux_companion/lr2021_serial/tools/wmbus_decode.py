#!/usr/bin/env python3
# Decode wM-Bus telegram headers (EN 13757) from the modem's "wmbus rx:" lines.
import sys, re, collections
TYPES = {0x00:"other",0x01:"oil",0x02:"electricity",0x03:"gas",0x04:"heat",0x05:"steam",0x06:"warm water",0x07:"water",
         0x08:"heat cost allocator",0x09:"compressed air",0x0a:"cooling (outlet)",0x0b:"cooling (inlet)",0x0c:"heat (inlet)",
         0x0d:"heat/cooling",0x0e:"bus/system",0x0f:"unknown",0x15:"hot water",0x16:"cold water",0x17:"dual water",
         0x18:"pressure",0x19:"A/D converter",0x1a:"smoke detector",0x1b:"room sensor",0x1c:"gas detector",0x20:"breaker",
         0x21:"valve",0x25:"display",0x28:"waste water",0x29:"garbage",0x30:"reserved",0x31:"comm controller",
         0x32:"unidirectional repeater",0x33:"bidirectional repeater",0x36:"radio converter (system)",0x37:"radio converter (meter)"}
def manuf(m):  # 2-byte little-endian manufacturer code -> 3 letters
    return "".join(chr(((m >> s) & 0x1f) + 64) for s in (10, 5, 0))
seen = collections.OrderedDict(); n = 0
cfg = ""
for line in open(sys.argv[1], errors="replace"):
    if line.startswith("wmbus cfg:"): cfg = line.strip()
    if not line.startswith("wmbus rx:"): continue
    b = bytes.fromhex(line.split(":",1)[1].strip()); n += 1
    if len(b) < 10: print("short:", b.hex()); continue
    L, C = b[0], b[1]; M = b[2] | (b[3] << 8); ident = b[4:8][::-1].hex(); ver, typ = b[8], b[9]
    key = (manuf(M), ident)
    m = re.search(r"rssi=(-?\d+)", cfg); rssi = int(m.group(1)) if m else None
    m = re.search(r"fmt=(\w)", cfg); fmt = m.group(1) if m else "?"
    m = re.search(r"mode=(\d+)", cfg); mode = m.group(1) if m else "?"
    d = seen.setdefault(key, {"n":0,"rssi":[], "type":TYPES.get(typ,f"0x{typ:02x}"), "ver":ver, "C":set(), "L":set(), "fmt":set(), "mode":set(), "sample":b.hex()[:40]})
    d["n"] += 1; d["C"].add(C); d["L"].add(L); d["fmt"].add(fmt); d["mode"].add(mode)
    if rssi is not None: d["rssi"].append(rssi)
print(f"{n} telegrams, {len(seen)} distinct meters")
print(f"{'manuf':6} {'id':9} {'type':22} {'ver':>3} {'n':>4} {'rssi(med)':>9} {'C-field':10} {'fmt':4} mode")
for (mf, ident), d in sorted(seen.items(), key=lambda kv: -kv[1]["n"]):
    r = sorted(d["rssi"]); med = r[len(r)//2] if r else "?"
    print(f"{mf:6} {ident:9} {d['type']:22} {d['ver']:3} {d['n']:4} {str(med):>9} {','.join(f'{c:02x}' for c in sorted(d['C'])):10} {'/'.join(sorted(d['fmt'])):4} {','.join(sorted(d['mode']))}")
