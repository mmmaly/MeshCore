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
