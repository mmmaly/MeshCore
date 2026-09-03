#!/usr/bin/env python3
# Three-receiver comparison on the live CZ channel: node kit (boost off),
# monitor kit (boost 7), Nooelec RTL-SDR via lora_rx. Packets are matched by
# identical payload bytes (same relayed copy). Kit logs carry only uptime
# ms, so their window is "the last WINDOW_MS of uptime" ending at the newest
# line; the SDR log is exactly the capture.
import re, sys, collections
WINDOW_MS = int(sys.argv[4]) if len(sys.argv) > 4 else 1800000

def parse(path, window=None):
    pk = {}
    cfg = None
    lines = open(path, errors="replace").read().splitlines()
    for l in lines:
        if l.startswith("rx cfg:"):
            cfg = l
        elif l.startswith("rx ok: ") and cfg:
            m = re.search(r"snr=(-?[\d.]+)", cfg); snr = float(m.group(1)) if m else None
            m = re.search(r"rssi=(-?[\d.]+)", cfg); rssi = float(m.group(1)) if m else None
            m = re.search(r"time=(\d+)", cfg); t = int(m.group(1)) if m else 0
            m = re.search(r" sf=(\d+)", cfg); sf = int(m.group(1)) if m else 0
            pk[l[7:].strip()] = (t, snr, rssi, sf)
            cfg = None
    if window and pk:
        tmax = max(v[0] for v in pk.values())
        pk = {k: v for k, v in pk.items() if v[0] >= tmax - window}
    return pk

kit1 = parse(sys.argv[1], WINDOW_MS)
kit2 = parse(sys.argv[2], WINDOW_MS)
sdr  = parse(sys.argv[3])
allp = set(kit1) | set(kit2) | set(sdr)
print(f"packets: kit1(node, boost off)={len(kit1)}  kit2(monitor, boost 7)={len(kit2)}  sdr={len(sdr)}  union={len(allp)}")
for name, a in (("kit1", kit1), ("kit2", kit2), ("sdr", sdr)):
    snrs = sorted(v[1] for v in a.values() if v[1] is not None)
    if snrs: print(f"  {name}: coverage {len(a)/len(allp)*100:.0f}%  snr median {snrs[len(snrs)//2]:.1f}  min {snrs[0]:.1f}")
both12 = set(kit1) & set(kit2)
if both12:
    d = [kit2[k][2] - kit1[k][2] for k in both12 if kit1[k][2] is not None and kit2[k][2] is not None]
    d.sort(); print(f"kit2 - kit1 RSSI on {len(d)} common packets: median {d[len(d)//2]:+.1f} dB")
    ds = [kit2[k][1] - kit1[k][1] for k in both12]
    ds.sort(); print(f"kit2 - kit1 SNR  on {len(ds)} common packets: median {ds[len(ds)//2]:+.1f} dB")
bothks = set(kit2) & set(sdr)
if bothks:
    ds = sorted(sdr[k][1] - kit2[k][1] for k in bothks)
    print(f"sdr - kit2 SNR on {len(ds)} common packets: median {ds[len(ds)//2]:+.1f} dB")
# what did each receiver miss that another heard, by the other's SNR
for name, a, b in (("kit1 missed (heard by kit2)", kit1, kit2), ("sdr missed (heard by kit2)", sdr, kit2), ("kit2 missed (heard by kit1)", kit2, kit1)):
    miss = [b[k][1] for k in b if k not in a and b[k][1] is not None]
    if miss:
        miss.sort(); print(f"  {name}: {len(miss)} packets, their SNR at the other receiver: median {miss[len(miss)//2]:.1f}, max {miss[-1]:.1f}")
sfs = collections.Counter(v[3] for v in kit2.values()); print("kit2 SF of received packets:", dict(sfs))
