# linux_companion — stock MeshCore companion node, SDR radio, Linux host

**Status: spike in progress.** The goal is to run the *unmodified* companion
firmware (`examples/companion_radio/MyMesh.cpp` and the `src/` mesh core) on
a Linux host, with a pair of SDRs standing in for the LoRa transceiver:

```
phone / desktop app ──TCP──> linux_companion ──┬── lora_rx (RTL-SDR)  receive
                                               └── lora_tx (HackRF)   transmit
```

Why: a hand-written reimplementation of the companion protocol has to chase
firmware byte-for-byte forever, and answers "what firmware are you on?" with
a shrug. Running the real sources means protocol fidelity is free, upstream
features arrive with a `git pull`, and the version is simply whatever the
tree says.

## How it fits

MeshCore already has the seams this needs:

| Seam | Provided here |
|---|---|
| `mesh::Radio` (abstract) | `SdrRadio` — drives `lora_rx`/`lora_tx` as child processes |
| `BaseSerialInterface` (abstract) | `TcpSerialInterface` — MeshCore's `0x3C/0x3E` framing over TCP |
| `mesh::MainBoard`, `mesh::RNG`, `mesh::RTCClock` | `LinuxPlatform.h` |
| Arduino / `AES128` / `SHA256` | `host/` shims (crypto over OpenSSL) |
| Ed25519 | the vendored `lib/ed25519` — the same implementation the MCU uses |

Nothing under `src/` or `examples/companion_radio/` is modified.

## Progress

- [x] `src/` core compiles unmodified on the host: `Mesh.cpp`, `Dispatcher.cpp`,
      `Packet.cpp`, `Utils.cpp`, `Identity.cpp`
- [x] `SdrRadio` (mesh::Radio over lora_rx/lora_tx)
- [x] `TcpSerialInterface` (BaseSerialInterface over TCP)
- [x] Linux board / RNG / RTC
- [ ] `DataStore` filesystem shim (`FILESYSTEM`/`File` over stdio)
- [ ] `MyMesh.cpp` compiling against the shims
- [ ] `main.cpp` wiring + build system
- [ ] Live test against the mesh

## Building the core (what works today)

```bash
g++ -std=c++17 -c src/Mesh.cpp -Isrc -Ilib/ed25519 \
    -Iexamples/linux_companion/host -I$(brew --prefix openssl)/include
```

## Result of the spike

It works. The stock companion firmware runs on a Linux host and answers the
companion protocol over TCP:

```
DEVICE_INFO: code=13 fw_ver=13 max_contacts=200 max_channels=8
  model='MeshCore SDR (RTL-SDR rx, HackRF tx)'  firmware='v1.17.1'
SELF_INFO:   code=5 pubkey=7a9a144e62447aa5... freq=915000 bw=250000 sf=10 cr=5
```

"What firmware are you on?" now has an honest answer: **v1.17.1**, the same
sources every other node runs.

The only change outside this directory is a two-line `LINUX_PLATFORM` branch
in `DataStore::formatFileSystem()`, where the platform switch previously
ended in `#error "need to implement format()"`. That is the shape an
upstream "add a Linux platform" change would take.

### It immediately found a real bug

Within minutes of running side by side with the hand-written reimplementation
this replaces, the two disagreed about `SELF_INFO`: firmware sends
`_prefs.freq * 1000` where `freq` is **MHz**, i.e. the field is **kHz** -
while bandwidth in the next field is `bw * 1000` from kHz, i.e. **Hz**. The
app matches firmware (`freqHz = (freqMHz * 1000).round()` - a variable whose
name says Hz while carrying kHz). The reimplementation had been sending Hz,
so it reported a frequency 1000x too large and would have retuned to
nonsense had anyone changed radio settings from the app.

That is the whole argument for this port in one bug: behaviour you mirror by
reading is behaviour you can get subtly wrong forever, and never notice.

## Second radio: the Semtech LR2021 EVK over USB serial (`SerialRadio`)

Since 2026-09-03 the same binary can drive a real LoRa transceiver instead of
the SDR pair: the **LR2021EVK1XBS1** kit (Seeed XIAO nRF54L15 + Wio-LR2021)
running the `lr2021_serial/` firmware in this directory. The kit is a
microcontroller board, not a computer, so the node still lives on the host:

```
app ──TCP──> linux_companion --serial /dev/ttyACM0 ──USB CDC──> lr2021_serial (nRF54L15) ──SPI──> LR2021
```

| Seam | Provided here |
|---|---|
| `HostRadio` (`mesh::Radio` + what `MyMesh` calls on `radio_driver`) | base of both `SdrRadio` and `SerialRadio` |
| `RadioProxy` | `radio_driver` global; `main()` selects the backend from argv, since `the_mesh` is constructed before argv is seen |
| `SerialRadio` | line protocol on the serial port: `rx cfg:`/`rx ok:` exactly as `lora_rx` prints them, `tx <hex>` → `tx done`, `set ...` for the app's radio settings |

```sh
LORA_FREQ=869.432 LORA_SF=7 sh examples/linux_companion/build.sh
./build/linux_companion -p 5000 -d ~/anaxos-cz --serial /dev/ttyACM0 --side-sfs 8 --tx-power 22
```

`--side-sfs` lists extra spreading factors the LR2021 receives *in parallel*
on the node's channel (hardware side detectors; all must be above the node's
SF and within +4 of it — the invalid ones are dropped when the app changes
the SF). `setParams` from the app retunes the modem live; the app's TX power
and "RX boosted gain" settings map onto the chip's PA (−9..+22 dBm,
`MAX_LORA_TX_POWER` in `build.sh`) and boosted-gain level. The port is
reopened and reconfigured after a USB glitch.
