#!/usr/bin/env python3
# TX power sweep: transmitter kit on TX_PORT, monitor kit on MON_PORT, both
# lr2021_serial. Tagged test packets on the clear test frequency; the monitor's
# per-packet RSSI/SNR is reported per requested power.
import os, sys, time, termios, re, collections

TX_PORT  = sys.argv[1]
MON_PORT = sys.argv[2]
FREQ     = int(sys.argv[3]) if len(sys.argv) > 3 else 869500000
POWERS   = [int(x) for x in sys.argv[4].split(",")] if len(sys.argv) > 4 else [-9, 0, 5, 10, 14, 18, 20, 22]
N        = 4

def open_port(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    a = termios.tcgetattr(fd)
    a[0] = 0; a[1] = 0; a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL; a[3] = 0
    a[4] = a[5] = termios.B115200
    a[6][termios.VMIN] = 0; a[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, a)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd

def send(fd, s): os.write(fd, (s + "\r\n").encode())

def read_lines(fd, buf, timeout):
    t0 = time.time(); out = []
    while time.time() - t0 < timeout:
        try: d = os.read(fd, 4096)
        except BlockingIOError: d = b""
        if d:
            buf[0] += d.decode("ascii", "replace")
            while "\n" in buf[0]:
                line, buf[0] = buf[0].split("\n", 1)
                out.append(line.strip("\r"))
        else:
            time.sleep(0.02)
    return out

tx, mon = open_port(TX_PORT), open_port(MON_PORT)
tbuf, mbuf = [""], [""]
def configure(fd, buf, name, line):
    for attempt in range(5):
        read_lines(fd, buf, 0.3)            # drain whatever the previous owner left
        send(fd, line)
        acks = [l for l in read_lines(fd, buf, 1.0) if l.startswith(("ok", "err"))]
        if any(l.startswith("ok cfg") for l in acks):
            print(name + ":", acks[-1]); return
        print(name, "retry", attempt, acks)
    sys.exit(name + ": configuration failed")
time.sleep(0.5)
configure(mon, mbuf, "mon", f"set freq={FREQ} sf=7 bw=62500 cr=5 sd=none boost=0")
configure(tx,  tbuf, "tx",  f"set freq={FREQ} sf=7 bw=62500 cr=5 sd=none pwr=0")

results = collections.OrderedDict()
for p in POWERS:
    send(tx, f"set pwr={p}")
    acks = [l for l in read_lines(tx, tbuf, 0.8) if l.startswith(("ok", "err"))]
    if not any(l.startswith("ok") for l in acks): print("pwr", p, "->", acks)
    got = []
    read_lines(mon, mbuf, 0.2)   # drain
    for i in range(N):
        payload = bytes([0xA5, (p + 100) & 0xFF, i]) + bytes(range(17))
        send(tx, "tx " + payload.hex())
        txl = read_lines(tx, tbuf, 1.0)
        lines = read_lines(mon, mbuf, 0.6)
        rssi = snr = None
        for l in lines:
            if l.startswith("rx cfg:"):
                m = re.search(r"snr=(-?[\d.]+) rssi=(-?[\d.]+)", l)
                if m: snr, rssi = float(m.group(1)), float(m.group(2))
            elif l.startswith("rx ok:") and l[7:].startswith(payload.hex()[:6]):
                got.append((rssi, snr))
        if not any(l.startswith("tx done") for l in txl): print("  tx", p, i, txl[-1:] )
        time.sleep(0.3)
    results[p] = got
    r = [g[0] for g in got if g[0] is not None]; s = [g[1] for g in got if g[1] is not None]
    print(f"pwr {p:>3} dBm: heard {len(got)}/{N}  rssi mean {sum(r)/len(r) if r else float('nan'):6.1f}  snr mean {sum(s)/len(s) if s else float('nan'):5.1f}  rssi {r}")

send(tx, "set freq=869432000 sd=8 pwr=22"); send(mon, "set freq=869432000 sd=8 boost=7")
time.sleep(0.5)
