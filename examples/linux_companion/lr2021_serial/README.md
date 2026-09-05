# lr2021_serial — LoRa modem firmware for the Semtech LR2021 EVK

Turns the **LR2021EVK1XBS1** kit (Seeed XIAO nRF54L15 + Wio-LR2021, the
"LoRa Plus" evaluation kit) into a LoRa modem that a Linux host drives over
the kit's USB serial port. It is the radio behind `SerialRadio` in the
parent directory: the stock MeshCore companion running on a Linux host, with
this board where the SDRs used to be.

```
linux_companion --serial /dev/ttyACM0 ──USB CDC──> SAMD11 bridge ──UART──> nRF54L15 ──SPI──> LR2021
```

The kit has no Linux-class computer on it — the nRF54L15 is a Cortex-M33
microcontroller — so the node logic stays on the host. What the chip brings:

- a real LoRa receiver (datasheet sensitivity −129 dBm at SF7/62.5 kHz), no
  ppm hunting, no deaf-open lottery;
- **side detectors**: up to three extra spreading factors received *in
  parallel* on the same channel (e.g. SF7 primary + SF8), which a single
  transceiver otherwise cannot do. Same frequency and bandwidth only — one
  channel at a time, unlike an SDR;
- a +22 dBm PA, versus roughly +14 dBm from a HackRF.

## Wire protocol (115200 8N1 on the SAMD11's CDC port)

Firmware → host, one line each:

| Line | Meaning |
|---|---|
| `rx cfg: freq=<Hz> sf=<n> bw=<Hz> cr=<5..8> snr=<dB> rssi=<dBm> det=<i> len=<n> time=<ms>` | a packet arrived; `sf` is the SF it was actually received on, `det` which detector caught it (0 = primary) |
| `rx ok: <hex>` | its CRC-valid payload (always right after `rx cfg:`, the same contract as `lora_rx`) |
| `rx err: ...` | a failed reception (header/CRC), for statistics |
| `tx done: len=<n> ms=<n>` / `tx err: <why>` | result of a `tx` |
| `ok [cfg: ...]` / `err <why>` | reply to a command |
| `cfg: freq=... sf=... bw=... cr=... sd=... pwr=... boost=... sync=... pre=...` | current configuration |

Host → firmware:

| Command | Effect |
|---|---|
| `tx <hex>` | transmit a raw LoRa frame (≤ 255 bytes) at the primary SF; blocks until `tx done` |
| `set k=v [k=v ...]` | any of `freq` (Hz), `sf`, `bw` (Hz), `cr` (5..8, or 1..4), `sd` (side-detector SFs, e.g. `8,9`, or `none`), `pwr` (dBm, −9..22), `boost` (RX boosted gain 0..7), `sync` (hex), `pre` (preamble symbols) |
| `status` | config plus counters (`rx`, `rx_err`, `tx`, `tx_err`), uptime, chip version |
| `rearm` / `reset` / `help` | re-arm the receiver / reboot / list commands |

Side-detector rules (chip): all extra SFs must be above the primary SF and
within +4 of it, the same header type and bandwidth as the primary. The
LR2021 *detects* several SFs at once but has one demodulator, so two
overlapping packets on different SFs still collide.

Defaults on boot are the Czech MeshCore preset (869.432 MHz, SF7, 62.5 kHz,
CR 4/5, sync 0x12, side detector SF8, +22 dBm, boost 7); the host re-sends
its own configuration on every connect anyway.

## Build

Mainline Zephyr (the `xiao_nrf54l15` board landed there in 4.2; tested on
4.4.99), a Zephyr SDK, and a RadioLib checkout (the commit MeshCore's
`platformio.ini` pins, or any release ≥ 7.7 with `setSideDetector`):

```sh
export ZEPHYR_BASE=$HOME/zephyrproject/zephyr RADIOLIB_DIR=$HOME/RadioLib
cd $HOME/zephyrproject
west build -p -b xiao_nrf54l15/nrf54l15/cpuapp -d build-lr2021 \
    /path/to/MeshCore/examples/linux_companion/lr2021_serial
```

## Flash

The XIAO's onboard SAMD11 enumerates as a CMSIS-DAP probe, so pyocd does it
(the target name is `nrf54l`; `-e chip` erases everything on the board,
including Semtech's LoRa Studio demo — back it up first with
`pyocd cmd -t nrf54l -c "savemem 0 0x17D000 backup.bin"` if you want it back):

```sh
pyocd flash -t nrf54l -e chip build-lr2021/zephyr/zephyr.hex
pyocd reset -t nrf54l
```

## Pin map (Wio-LR2021 on the XIAO connector)

| Signal | XIAO | nRF54L15 |
|---|---|---|
| SCK / MOSI / MISO | D8 / D10 / D9 | P2.01 / P2.02 / P2.04 (`spi00`) |
| NSS | D3 | P1.07 |
| IRQ (LR2021 DIO8) | D0 | P1.04 |
| RESET | D2 | P1.06 |
| BUSY | D1 | P1.05 |

Switchless RF path, 32 MHz crystal (TCXO mode is not enabled). The HAL and
the chip quirks (standby before re-arming RX, re-asserting the 255-byte RX
length after a transmit) follow MeshCore's `CustomLR2021` driver and the
XIAO nRF54L15 companion port proposed in meshcore-dev/MeshCore#2944.

## PA table: measured

Two kits 50 cm apart, monitor on the same channel with boosted gain off,
`tools/txtest.py` stepping the transmitter's requested power (4 tagged
packets per step, monitor RSSI in dBm, ±0.5 dB spread). Same geometry for
both firmware builds (`-DLR2021_STOCK_PA_TABLE=ON` selects RadioLib's table):

| requested | RadioLib stock table | Semtech Wio-LR2021 table |
|---|---|---|
| −9 dBm | −54.5 | −60.5 |
| 0 | −48.5 | −50.5 |
| 5 | −43.8 | −46.0 |
| 10 | −38.5 | −41.5 |
| 14 | −35.5 | −36.5 |
| 18 | −34.5 | −32.5 |
| 20 | −34.0 | −30.8 |
| 22 | −33.5 | −28.5 |

RadioLib's table (tuned for minimum current) flattens above 14 dBm — the
last 8 dB requested buy 2 dB on air; Semtech's keeps 1 dB/dB to the top and
is 5 dB stronger at 22 dBm. On the mesh that was the difference between a
node nobody heard (three flood adverts at 20 dBm, zero observer copies on
analyzer.meshcore.cz) and one heard by 53 observer copies on the next advert.
Absolute levels are relative to this geometry, not calibrated.

## Coding rates beyond classic LoRa

`set cr=` also takes the LR2021's long-interleaver codes `5li`, `6li`, `8li` and
the generation-4 convolutional codes `6cc`, `8cc` (chip codes 5..9; only
another LR2021 can decode them). Measured on the bench (two kits, no
antennas, 869.5 MHz / 1 MHz / SF5, 60 packets per mode at the same weak level
where 1 dB separates nothing from everything):

| coding rate | received |
|---|---|
| 4/5 | 0 / 60 |
| 4/5 long interleaver | 2 / 60 |
| 4/8 | 17 / 60 |
| 4/8 long interleaver | 11 / 60 |
| 4/6 convolutional | 16 / 60 |
| 4/8 convolutional | 39 / 60 |

The long interleaver adds nothing against plain noise (it targets burst
errors); the convolutional 4/8 code is worth about a decibel over standard
4/8 at the same airtime, and 4/6 convolutional matches standard 4/8 at 25%
less airtime.
