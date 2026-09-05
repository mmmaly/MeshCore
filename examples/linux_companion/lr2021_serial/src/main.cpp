/*
 * lr2021_serial: LoRa modem firmware for the Seeed XIAO nRF54L15 + Wio-LR2021
 * (Semtech LR2021 EVK), driven by a Linux host over the console UART.
 *
 * The host sees a line protocol on the SAMD11 bridge's USB CDC port
 * (115200 8N1). Every line the firmware prints starts with a keyword:
 *
 *   rx cfg: freq=<Hz> sf=<n> bw=<Hz> cr=<5..8> snr=<dB> rssi=<dBm> det=<i> len=<n> time=<ms>
 *   rx ok: <hex>              a CRC-valid packet; always follows its "rx cfg:" line
 *   rx err: ...               a reception that failed (CRC/header), for statistics
 *   tx done: len=<n> ms=<n>   / tx err: <reason>
 *   ok [cfg: ...]             / err <reason>      reply to a command
 *   cfg: freq=... sf=... bw=... cr=... sd=<sf,..|none> pwr=<dBm> boost=<0..7> sync=<hex> pre=<n>
 *
 * Commands from the host (one per line):
 *   tx <hex>                  transmit a raw LoRa payload (<= 255 bytes) at the primary SF
 *   set k=v [k=v ...]         keys as in the cfg line; sd=8,9 sets side detectors (extra SFs
 *                             received in parallel on the same channel, must all be > sf)
 *   status | reset | help
 *
 * "rx cfg:" + "rx ok:" deliberately mirror lora_rx's stdout contract so the
 * host side (SerialRadio.cpp) parses both radios the same way. The SF in the
 * rx cfg line is the one the packet actually arrived on (det=0 primary,
 * det=1..3 side detectors).
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/sys/ring_buffer.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <math.h>
#include "zephyr_hal.h"
#include "lr2021_pram.h"

#define FW_NAME "lr2021_serial"
#define FW_VERSION "1.0"

#ifndef LR2021_IRQ_DIO
#define LR2021_IRQ_DIO 8          // Wio-LR2021 wires DIO8 to the host IRQ line
#endif

static ZephyrHal hal;
static Module mod(&hal, LR_PIN_NSS, LR_PIN_IRQ, LR_PIN_RESET, LR_PIN_BUSY);
// RadioLib keeps the LoRa modulation codes it last programmed in protected
// members; the RTToF setup needs them to re-program the same modulation under
// the ranging packet type.
class Lr2021Radio : public LR2021 {
public:
  using LR2021::LR2021;
  uint8_t sfCode() const { return spreadingFactor; }
  uint8_t bwCode() const { return bandwidth; }
  uint8_t crCode() const { return codingRate; }
  uint8_t ldroCode() const { return ldrOptimize; }
  // raw command access for the modems RadioLib does not wrap (wM-Bus)
  int16_t cmdWrite(uint16_t op, const uint8_t* d, size_t n) { return SPIcommand(op, true, const_cast<uint8_t*>(d), n); }
  int16_t cmdRead(uint16_t op, uint8_t* buf, size_t n) { return SPIcommand(op, false, buf, n); }
};
static Lr2021Radio radio(&mod);

// Sub-GHz PA settings measured by Semtech on this module (usp_zephyr,
// semtech_wio_lr20xx_common.dtsi "tx-power-cfg-lf", meas @902 MHz), indexed
// by requested dBm + 9 as RadioLib expects. RadioLib's built-in table is
// tuned for minimum current and programs the PA several dB softer at the top
// (+22 dBm: tx_power 35 vs 44 here). Fields: {duty cycle, slices, tx_power}.
static LR2021PaTableEntry_t wio_pa_lf[32] = {
  {2, 5, -13}, {6, 1, -13}, {6, 0, -6}, {1, 0, 4},   // -9 .. -6 dBm
  {2, 0, 4},   {1, 3, 2},   {0, 0, 14}, {0, 3, 9},   // -5 .. -2
  {3, 0, 11},  {1, 0, 16},  {7, 0, 11}, {2, 0, 18},  // -1 .. 2
  {5, 0, 16},  {7, 0, 17},  {1, 2, 21}, {3, 0, 25},  // 3 .. 6
  {0, 1, 32},  {2, 0, 32},  {3, 1, 27}, {2, 1, 32},  // 7 .. 10
  {5, 1, 28},  {5, 1, 30},  {4, 1, 34}, {5, 4, 31},  // 11 .. 14
  {4, 4, 34},  {5, 6, 34},  {3, 5, 39}, {6, 6, 37},  // 15 .. 18
  {5, 5, 40},  {7, 4, 41},  {7, 4, 43}, {7, 7, 44},  // 19 .. 22
};


// 2.4 GHz PA settings Semtech measured on the Wio-LR2021 (same dtsi,
// "tx-power-cfg-hf", meas @2445 MHz): {half-dB power code, PA HF duty cycle},
// indexed by requested dBm + 18. RadioLib's setOutputPower() cannot be used on
// this path: it sends the LF duty-cycle field as its "unused" marker (6) where
// the chip wants 0 for the HF PA, and answers CMD_PERR.
struct HfPaEntry { int8_t half_power; uint8_t duty; };
static const HfPaEntry wio_pa_hf[31] = {
  {-39,29}, {-39,29}, {-39,16}, {-35,19}, {-32,19}, {-29,19}, {-27,16}, {-24,17},  // -18 .. -11
  {-22,16}, {-19,18}, {-17,16}, {-14,21}, {-12,18}, { -7,30}, { -8,16}, { -5,24},  // -10 .. -3
  { -2,27}, {  1,29}, {  4,30}, {  6,30}, {  7,28}, {  8,25}, { 10,25}, { 15,31},  //  -2 .. 5
  { 16,30}, { 18,30}, { 21,31}, { 22,30}, { 24,30}, { 24,26}, { 24,16},            //   6 .. 12
};


struct Config {
  uint32_t freq_hz = 869432000;   // MeshCore CZ community preset
  uint8_t  sf = 7;
  uint32_t bw_hz = 62500;
  uint8_t  cr = 5;                // 4/5 .. 4/8, RadioLib convention (standard codes)
  uint8_t  cr_code = 1;           // chip code actually programmed: 1..4 standard, 5/6/7 = 4/5,4/6,4/8 long
                                  // interleaver, 8/9 = 4/6,4/8 convolutional (LR2021-only, "LoRa Plus")
  uint8_t  side[3] = {8, 0, 0};   // side-detector SFs, all > sf, span <= 4
  uint8_t  nside = 1;
  int8_t   pwr = 22;              // dBm; the sub-GHz PA goes to +22
  uint8_t  boost = 7;             // RX boosted gain level (Semtech usp default 7)
  uint8_t  sync = 0x12;           // private network sync word (MeshCore)
  uint16_t pre = 16;              // preamble symbols on TX (MeshCore uses 16)
};
static Config cfg;

static int16_t set_output_power(int8_t pwr) {
  if (cfg.freq_hz <= 1500000000u) return radio.setOutputPower(pwr);   // LF: RadioLib + wio_pa_lf
  const HfPaEntry& e = wio_pa_hf[pwr + 18];
  // pa_sel=HF, pa_lf_mode=FSM, pa_lf_duty=0, pa_lf_slices=7, pa_hf_duty=table (as Semtech's BSP does)
  // Order matters (measured on fw 1.24): SetPaConfig selecting the HF PA answers
  // CMD_PERR while the TX power still programmed is outside the HF range (e.g. the
  // 44 = +22 dBm left over from sub-GHz operation). Program the HF power first.
  int16_t st = radio.setTxParams(e.half_power, RADIOLIB_LRXXXX_PA_RAMP_48U);
  if (st != RADIOLIB_ERR_NONE) { printk("err hf setTxParams %d (half %d)\n", st, e.half_power); return st; }
  st = radio.setPaConfig(1, RADIOLIB_LR2021_PA_LF_MODE_FSM, 0, 7, e.duty);
  if (st != RADIOLIB_ERR_NONE) printk("err hf setPaConfig %d (duty %u)\n", st, e.duty);
  return st;
}

static uint32_t n_rx = 0, n_rx_err = 0, n_tx = 0, n_tx_err = 0;
static int64_t last_rx_ms = 0;
static volatile bool irq_flag = false;
static void on_irq() { irq_flag = true; }

/* ---------- UART input: ISR fills a ring buffer, main loop assembles lines ---------- */
static const device* uart_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
RING_BUF_DECLARE(uart_rb, 2048);

static void uart_isr(const device* dev, void*) {
  uart_irq_update(dev);
  while (uart_irq_rx_ready(dev)) {
    uint8_t buf[64];
    int n = uart_fifo_read(dev, buf, sizeof(buf));
    if (n <= 0) break;
    ring_buf_put(&uart_rb, buf, n);
  }
}

/* ---------- helpers ---------- */
static const char* cr_name(uint8_t code) {
  static const char* n[] = {"?", "5", "6", "7", "8", "5li", "6li", "8li", "6cc", "8cc"};
  return code <= 9 ? n[code] : "?";
}
static int cr_code_from(const char* v) {
  if (!strcmp(v, "5li")) return 5; if (!strcmp(v, "6li")) return 6; if (!strcmp(v, "8li")) return 7;
  if (!strcmp(v, "6cc")) return 8; if (!strcmp(v, "8cc")) return 9;
  int cr = atoi(v); if (cr >= 1 && cr <= 4) cr += 4;
  return (cr >= 5 && cr <= 8) ? cr - 4 : -1;
}

static void print_cfg(const char* prefix) {
  char sd[16] = "none";
  if (cfg.nside) {
    int p = 0;
    for (int i = 0; i < cfg.nside; i++) p += snprintf(sd + p, sizeof(sd) - p, "%s%u", i ? "," : "", cfg.side[i]);
  }
  printk("%scfg: freq=%u sf=%u bw=%u cr=%s sd=%s pwr=%d boost=%u sync=%02x pre=%u\n",
         prefix, cfg.freq_hz, cfg.sf, cfg.bw_hz, cr_name(cfg.cr_code), sd, cfg.pwr, cfg.boost, cfg.sync, cfg.pre);
}

// Semtech airtime formula (ms), explicit header, CRC on.
static uint32_t airtime_ms(int len, uint8_t sf, uint32_t bw, uint8_t cr) {
  double tsym = (double)(1u << sf) / bw;
  double nsym = cfg.pre + 4.25 + 8 + fmax(0.0, ceil((8.0 * len - 4.0 * sf + 44) / (4.0 * sf)) * cr);
  return (uint32_t)(nsym * tsym * 1000.0);
}

static int16_t arm_rx() {
  // The LR2021 answers a re-arm from continuous RX with CMD_PERR; drop to
  // standby first. explicitHeader() rewrites the packet params with the
  // 255-byte max length, which a preceding TX shrank to that packet's size.
  radio.standby();
  int16_t st = radio.explicitHeader();
  if (st == RADIOLIB_ERR_NONE) st = radio.startReceive();
  if (st != RADIOLIB_ERR_NONE) printk("err rx arm %d\n", st);
  return st;
}

static int16_t apply_config() {
  radio.standby();
  // The sub-GHz PA spans -9..+22 dBm, the 2.4 GHz one -18..+12; RadioLib rejects
  // anything outside the band's range, so clamp the request rather than fail the
  // whole configuration when the app carries a 22 dBm setting over to 2.4 GHz.
  int8_t pwr = cfg.pwr;
  if (cfg.freq_hz > 1500000000u) { if (pwr > 12) pwr = 12; if (pwr < -18) pwr = -18; }
  else                           { if (pwr > 22) pwr = 22; if (pwr < -9)  pwr = -9;  }
  struct Step { const char* name; int16_t st; } steps[] = {
    {"frequency", radio.setFrequency(cfg.freq_hz / 1000000.0f)},
    {"bandwidth", RADIOLIB_ERR_NONE}, {"sf", RADIOLIB_ERR_NONE}, {"cr", RADIOLIB_ERR_NONE},
    {"sync", RADIOLIB_ERR_NONE}, {"preamble", RADIOLIB_ERR_NONE}, {"power", RADIOLIB_ERR_NONE},
    {"boost", RADIOLIB_ERR_NONE}, {"crc", RADIOLIB_ERR_NONE}, {"header", RADIOLIB_ERR_NONE},
  };
  int16_t st = steps[0].st;
  if (st == RADIOLIB_ERR_NONE) st = steps[1].st = radio.setBandwidth(cfg.bw_hz / 1000.0f);
  if (st == RADIOLIB_ERR_NONE) st = steps[2].st = radio.setSpreadingFactor(cfg.sf);
  if (st == RADIOLIB_ERR_NONE) st = steps[3].st = radio.setCodingRate(cfg.cr);
  if (st == RADIOLIB_ERR_NONE) st = steps[4].st = radio.setSyncWord(cfg.sync);
  if (st == RADIOLIB_ERR_NONE) st = steps[5].st = radio.setPreambleLength(cfg.pre);
  if (st == RADIOLIB_ERR_NONE) st = steps[6].st = set_output_power(pwr);
  if (st == RADIOLIB_ERR_NONE) st = steps[7].st = radio.setRxBoostedGainMode(cfg.boost);
  if (st == RADIOLIB_ERR_NONE) st = steps[8].st = radio.setCRC(2);
  if (st == RADIOLIB_ERR_NONE) st = steps[9].st = radio.explicitHeader();
  if (st != RADIOLIB_ERR_NONE) {
    const char* which = "?";
    for (auto& s : steps) if (s.st != RADIOLIB_ERR_NONE) { which = s.name; break; }
    printk("err modulation %d at %s\n", st, which);
    return st;
  }

  // LoRa Plus coding rates (long interleaver / convolutional): RadioLib's cached
  // code stays at the nearest standard one; program the real code directly.
  if (cfg.cr_code > 4) {
    st = radio.setLoRaModulationParams(radio.sfCode(), radio.bwCode(), cfg.cr_code, radio.ldroCode());
    if (st != RADIOLIB_ERR_NONE) { printk("err coding rate code %u: %d\n", cfg.cr_code, st); return st; }
  }
  // Side detectors last: SetLoraModulationParams (any of the setters above)
  // clears them in the chip. Same channel and bandwidth, own SF/LDRO/sync.
  if (cfg.nside) {
    LR2021LoRaSideDetector_t sd[3];
    for (int i = 0; i < cfg.nside; i++) {
      sd[i].sf = cfg.side[i];
      sd[i].ldro = ((float)(1u << sd[i].sf) / (cfg.bw_hz / 1000.0f)) >= 16.0f;  // tsym >= 16 ms
      sd[i].invertIQ = false;
      sd[i].syncWord = cfg.sync;
    }
    st = radio.setSideDetector(sd, cfg.nside);
  } else {
    st = radio.setSideDetector(nullptr, 0);
  }
  if (st != RADIOLIB_ERR_NONE) { printk("err side detectors %d\n", st); return st; }
  return arm_rx();
}


/* ---------- RTToF ranging (LoRa Plus "time of flight") ----------
 * Sequence and calibration mirror Semtech's USP ranging demo (smtc_rac_lora.c,
 * app_ranging_hopping.c): packet type RTToF, LoRa modulation params, the PLL
 * frequency-step register workaround, the per-BW/SF Tx->Rx delay indicator,
 * then the manager sends a request (SetTx) and the subordinate answers from
 * continuous RX. Distance [m] = raw * 150 / (4096 * BW_MHz).
 */
static const uint32_t rng_delay_below_600m[7][8] = {
  {19737,19694,19614,19457,19159,18632,19036,19024}, {17502,17546,17566,17682,17739,18042,19036,19024},
  {20134,20111,20068,19981,19811,19489,20236,20232}, {17794,17827,17831,17871,17819,17826,20295,20298},
  {20569,20579,20577,20549,20491,20372,20295,20298}, {18713,18778,18746,18805,18725,18786,20295,20298},
  {21629,21660,21685,21660,21597,21466,20295,20298},
};
static const uint32_t rng_delay_600m_2g[7][8] = {
  {19747,19707,19628,19480,19166,18589,19036,19024}, {17498,17502,17515,17606,17722,18024,19036,19024},
  {20150,20133,20102,20033,19847,19537,20236,20232}, {17768,17791,17868,17997,18123,18456,20295,20298},
  {20599,20590,20567,20512,20295,19961,20295,20298}, {18681,18738,18763,18874,18737,18824,20295,20298},
  {21700,21705,21783,21834,21689,21571,20295,20298},
};
static const uint32_t rng_delay_above_2g[7][8] = {
  {19582,19498,19330,19012,18368,17125,19036,19024}, {17173,17262,17335,17554,17828,18557,19036,19024},
  {19938,19896,19818,19646,19316,18667,20236,20232}, {17767,17822,17869,17937,18119,18442,20295,20298},
  {20588,20586,20550,20451,20287,19938,20295,20298}, {18698,18777,18848,18981,19047,19449,20295,20298},
  {21574,21611,21622,20095,21370,21009,20295,20298},
};
static int rng_bw_row(uint32_t bw) {
  switch (bw) { case 125000: return 0; case 203125: return 1; case 250000: return 2; case 406250: return 3;
                case 500000: return 4; case 812500: return 5; case 1000000: return 6; default: return -1; }
}
static bool rng_mode = false;        // radio is in RTToF packet type (LoRa RX suspended)
static bool rng_subordinate = false;
static uint32_t rng_user_delay = 0;  // 0 = Semtech's table value
static uint32_t rng_addr = 0x32101222;
static int64_t rng_sub_until = 0;    // uptime ms at which an unattended subordinate window ends (0 = none)

static uint32_t rng_delay_indicator() {
  if (rng_user_delay) return rng_user_delay;
  int row = rng_bw_row(cfg.bw_hz); int col = cfg.sf - 5;
  if (row < 0 || col < 0 || col > 7) return 0;
  if (cfg.freq_hz < 600000000u) return rng_delay_below_600m[row][col];
  if (cfg.freq_hz < 2000000000u) return rng_delay_600m_2g[row][col];
  return rng_delay_above_2g[row][col];
}

// Common part of Semtech's manager/subordinate setup; the LoRa modulation
// codes come from RadioLib's cache filled by apply_config() (LoRa mode).
static int16_t rng_setup(bool subordinate, uint32_t addr) {
  if (rng_bw_row(cfg.bw_hz) < 0) { printk("err rng: bandwidth %u not supported for ranging (125k..1M)\n", cfg.bw_hz); return -1; }
  radio.standby();
  int16_t st = radio.setPacketType(RADIOLIB_LR2021_PACKET_TYPE_RTTOF);
  if (st == RADIOLIB_ERR_NONE) st = radio.setLoRaModulationParams(radio.sfCode(), radio.bwCode(), radio.crCode(), radio.ldroCode());
  if (st == RADIOLIB_ERR_NONE) {          // workaround: truncate the PLL frequency step (reg 0x00F40144 &= ~0x7F)
    uint32_t v = 0; st = radio.readRegMem32(0x00F40144, &v, 1);
    if (st == RADIOLIB_ERR_NONE) { v &= ~0x7Fu; st = radio.writeRegMem32(0x00F40144, &v, 1); }
  }
  if (st == RADIOLIB_ERR_NONE && !subordinate) st = set_output_power(cfg.pwr);
  if (st == RADIOLIB_ERR_NONE) st = radio.setLoRaPacketParams(12, RADIOLIB_LRXXXX_LORA_HEADER_EXPLICIT, 7, RADIOLIB_LRXXXX_LORA_CRC_ENABLED, 0);
  if (st == RADIOLIB_ERR_NONE) st = radio.setLoRaSyncword(cfg.sync);
  uint32_t irq = subordinate ? (RADIOLIB_LR2021_IRQ_TIMEOUT | RADIOLIB_LR2021_IRQ_RNG_REQ_DIS | RADIOLIB_LR2021_IRQ_RNG_RESP_DONE | RADIOLIB_LR2021_IRQ_RNG_REQ_VALID)
                             : (RADIOLIB_LR2021_IRQ_RNG_EXCH_VALID | RADIOLIB_LR2021_IRQ_RNG_TIMEOUT | RADIOLIB_LR2021_IRQ_TIMEOUT);
  if (st == RADIOLIB_ERR_NONE) st = radio.setDioIrqConfig(LR2021_IRQ_DIO, irq);
  if (st == RADIOLIB_ERR_NONE) st = radio.setRangingTxRxDelay(rng_delay_indicator());
  if (st == RADIOLIB_ERR_NONE) st = radio.setRangingParams(false, 15);   // 15 response symbols (Semtech default)
  if (st == RADIOLIB_ERR_NONE) st = subordinate ? radio.setRangingAddr(addr, 4) : radio.setRangingReqAddr(addr);
  if (st == RADIOLIB_ERR_NONE) st = radio.clearIrqState(RADIOLIB_LR2021_IRQ_ALL);
  if (st != RADIOLIB_ERR_NONE) { printk("err rng setup %d\n", st); return st; }
  rng_mode = true; rng_subordinate = subordinate; rng_addr = addr;
  return st;
}

static int16_t rng_arm_subordinate() {
  radio.clearIrqState(RADIOLIB_LR2021_IRQ_ALL);
  return radio.setRx(RADIOLIB_LR2021_RX_TIMEOUT_INF);
}

static void rng_off() {
  rng_mode = false; rng_subordinate = false; rng_sub_until = 0;
  radio.standby();
  radio.clearTxFifo(); radio.clearRxFifo();
  radio.setPacketType(RADIOLIB_LR2021_PACKET_TYPE_LORA);
  apply_config();
}

// One manager exchange: returns true and fills raw/rssi on a valid response.
static bool rng_exchange(int32_t* raw, uint8_t* rssi, uint32_t* irq_out) {
  static const uint8_t payload[7] = {0x52, 0x4e, 0x47, 0x00, 0x00, 0x00, 0x00};
  radio.standby();
  radio.clearIrqState(RADIOLIB_LR2021_IRQ_ALL);
  radio.clearTxFifo();                       // stale bytes here corrupt the next TX (found the hard way)
  radio.writeRadioTxFifo(payload, sizeof(payload));
  int16_t st = radio.setTx(0);
  if (st != RADIOLIB_ERR_NONE) { printk("err rng setTx %d\n", st); return false; }
  int64_t t0 = k_uptime_get();
  uint32_t irq = 0;
  while (k_uptime_get() - t0 < 1500) {
    irq = radio.getIrqFlags();
    if (irq & (RADIOLIB_LR2021_IRQ_RNG_EXCH_VALID | RADIOLIB_LR2021_IRQ_RNG_TIMEOUT | RADIOLIB_LR2021_IRQ_TIMEOUT)) break;
    k_msleep(1);
  }
  *irq_out = irq;
  if (!(irq & RADIOLIB_LR2021_IRQ_RNG_EXCH_VALID)) { radio.clearIrqState(RADIOLIB_LR2021_IRQ_ALL); return false; }
  uint32_t r1 = 0, r2 = 0; uint8_t rs = 0;
  radio.getRangingResult(RADIOLIB_LR2021_RANGING_RESULT_TYPE_RAW, &r1, &rs, &r2);
  radio.clearIrqState(RADIOLIB_LR2021_IRQ_ALL);
  int32_t v = (int32_t)r1; if (v >= (1 << 23)) v -= (1 << 24);   // signed 24-bit
  *raw = v; *rssi = rs;
  return true;
}

static float rng_metres(int32_t raw) { return (float)raw * 150.0f / (4096.0f * (cfg.bw_hz / 1000000.0f)); }


/* ---------- wM-Bus reception (EN 13757-4), the LR2021's own modem ----------
 * The chip demodulates, decodes 3-of-6 / Manchester, detects frame format A/B
 * and checks every CRC block; we get the telegram bytes and a status.
 */
static bool wmbus_mode = false;
static uint8_t wmbus_chip_mode = 0;
static uint32_t n_wmbus = 0;

static int16_t wmbus_arm() {
  radio.clearRxFifo();
  radio.clearIrqState(RADIOLIB_LR2021_IRQ_ALL);
  return radio.setRx(RADIOLIB_LR2021_RX_TIMEOUT_INF);
}

static int16_t wmbus_start(uint8_t mode, uint32_t freq_hz) {
  radio.standby();
  int16_t st = radio.setPacketType(RADIOLIB_LR2021_PACKET_TYPE_WM_BUS);
  if (st == RADIOLIB_ERR_NONE) st = radio.setFrequency(freq_hz / 1000000.0f);
  // SetWmbusParams: mode, rx_bw auto, format A (B is auto-detected), no address filter,
  // max L-field 255, TX preamble 32 bits, RX preamble detect auto
  uint8_t prm[8] = {mode, 0xFF, 0x00, 0x00, 0xFF, 0x00, 0x20, 0xFF};
  if (st == RADIOLIB_ERR_NONE) st = radio.cmdWrite(0x026A, prm, sizeof(prm));
  if (st == RADIOLIB_ERR_NONE) st = radio.setRxBoostedGainMode(cfg.boost);
  if (st == RADIOLIB_ERR_NONE) st = radio.setDioIrqConfig(LR2021_IRQ_DIO,
      RADIOLIB_LR2021_IRQ_RX_DONE | RADIOLIB_LR2021_IRQ_CRC_ERROR | RADIOLIB_LR2021_IRQ_LEN_ERROR | RADIOLIB_LR2021_IRQ_TIMEOUT);
  if (st == RADIOLIB_ERR_NONE) st = wmbus_arm();
  if (st != RADIOLIB_ERR_NONE) { printk("err wmbus start %d\n", st); return st; }
  wmbus_mode = true; wmbus_chip_mode = mode;
  return st;
}

static void wmbus_stop() {
  wmbus_mode = false;
  radio.standby();
  radio.clearRxFifo(); radio.clearTxFifo();
  radio.setPacketType(RADIOLIB_LR2021_PACKET_TYPE_LORA);
  apply_config();
}

static void wmbus_service(uint32_t irq) {
  if (!(irq & (RADIOLIB_LR2021_IRQ_RX_DONE | RADIOLIB_LR2021_IRQ_CRC_ERROR | RADIOLIB_LR2021_IRQ_LEN_ERROR | RADIOLIB_LR2021_IRQ_TIMEOUT))) return;
  uint8_t stb[9] = {0};
  radio.cmdRead(0x026D, stb, sizeof(stb));               // GetWmbusPacketStatus (parsed as Semtech's driver does)
  unsigned lfield = stb[0];
  unsigned len = ((unsigned)stb[1] << 8) | stb[2];
  int rssi = -(int)stb[3];
  uint32_t crcerr = ((uint32_t)stb[5] << 9) | ((uint32_t)stb[6] << 1) | ((stb[7] >> 6) & 1);
  char fmt = ((stb[7] >> 7) & 1) ? 'B' : 'A';
  unsigned lqi = stb[8];
  static uint8_t buf[300];
  if (len > sizeof(buf)) len = sizeof(buf);
  if ((irq & RADIOLIB_LR2021_IRQ_RX_DONE) && len > 0) {
    radio.readRadioRxFifo(buf, len);
    static char hex[2 * 300 + 1];
    for (unsigned i = 0; i < len; i++) snprintf(hex + 2 * i, 3, "%02x", buf[i]);
    printk("wmbus rx: %s\n", hex);
    printk("wmbus cfg: mode=%u fmt=%c lfield=%u len=%u rssi=%d lqi=%u crcerr=%08x time=%lld\n",
           wmbus_chip_mode, fmt, lfield, len, rssi, lqi, crcerr, k_uptime_get());
    n_wmbus++;
  } else {
    printk("wmbus err: irq=%08x len=%u rssi=%d crcerr=%08x\n", irq, len, rssi, crcerr);
  }
  wmbus_arm();
}

/* ---------- radio service: called on IRQ edge and as a periodic poll ---------- */
static void service_radio() {
  uint32_t irq = radio.getIrqFlags();
  if (!irq) return;
  if (wmbus_mode) { wmbus_service(irq); return; }
  if (rng_mode) {
    if (rng_subordinate && (irq & (RADIOLIB_LR2021_IRQ_RNG_RESP_DONE | RADIOLIB_LR2021_IRQ_RNG_REQ_DIS | RADIOLIB_LR2021_IRQ_TIMEOUT))) {
      printk("rng sub: irq=%08x %s\n", irq, (irq & RADIOLIB_LR2021_IRQ_RNG_RESP_DONE) ? "response sent" : "request discarded/timeout");
      rng_arm_subordinate();
    }
    return;                 // manager exchanges are driven synchronously from the command
  }

  if (irq & RADIOLIB_LR2021_IRQ_RX_DONE) {
    uint8_t cr = 0, det = 0, plen = 0; bool crc = false;
    float snr = 0, rssi = 0, rssi_sig = 0;
    radio.getLoRaPacketStatus(&cr, &crc, &plen, &snr, &rssi, &rssi_sig, &det);
    size_t len = radio.getPacketLength();
    static uint8_t buf[RADIOLIB_LR2021_MAX_PACKET_LENGTH + 1];
    if (len > RADIOLIB_LR2021_MAX_PACKET_LENGTH) len = RADIOLIB_LR2021_MAX_PACKET_LENGTH;
    int16_t st = radio.readData(buf, len);     // also clears the IRQ flags
    unsigned sf_rx = (det >= 1 && det <= cfg.nside) ? cfg.side[det - 1] : cfg.sf;
    if (st == RADIOLIB_ERR_NONE && len > 0) {
      static char hex[2 * RADIOLIB_LR2021_MAX_PACKET_LENGTH + 1];
      for (size_t i = 0; i < len; i++) snprintf(hex + 2 * i, 3, "%02x", buf[i]);
      // the chip reports the header's CR code (1..4 = 4/5..4/8); print it RadioLib-style
      printk("rx cfg: freq=%u sf=%u bw=%u cr=%s snr=%.1f rssi=%.1f det=%u len=%u time=%lld\n",
             cfg.freq_hz, sf_rx, cfg.bw_hz, cr_name(cr), (double)snr, (double)rssi, det,
             (unsigned)len, k_uptime_get());
      printk("rx ok: %s\n", hex);
      n_rx++;
      last_rx_ms = k_uptime_get();
    } else {
      printk("rx err: st=%d sf=%u len=%u snr=%.1f rssi=%.1f\n", st, sf_rx, (unsigned)len,
             (double)snr, (double)rssi);
      n_rx_err++;
    }
    arm_rx();
    return;
  }
  if (irq & (RADIOLIB_LR2021_IRQ_LORA_HDR_CRC_ERROR | RADIOLIB_LR2021_IRQ_CRC_ERROR |
             RADIOLIB_LR2021_IRQ_TIMEOUT | RADIOLIB_LR2021_IRQ_LEN_ERROR |
             RADIOLIB_LR2021_IRQ_ERROR | RADIOLIB_LR2021_IRQ_CMD_ERROR)) {
    printk("rx err: irq=%08x\n", irq);
    n_rx_err++;
    radio.clearIrqFlags(RADIOLIB_LR2021_IRQ_ALL);
    arm_rx();
    return;
  }
  // PREAMBLE_DETECTED / HEADER_VALID etc.: a packet is in flight, leave it alone.
}

static void do_tx(const uint8_t* data, int len) {
  radio.standby();
  int64_t t0 = k_uptime_get();
  int16_t st = radio.startTransmit(data, len);
  if (st != RADIOLIB_ERR_NONE) {
    printk("tx err: start %d\n", st);
    n_tx_err++;
    arm_rx();
    return;
  }
  uint32_t limit = airtime_ms(len, cfg.sf, cfg.bw_hz, cfg.cr) * 2 + 1000;
  bool done = false;
  while (k_uptime_get() - t0 < limit) {
    if (radio.getIrqFlags() & RADIOLIB_LR2021_IRQ_TX_DONE) { done = true; break; }
    k_msleep(1);
  }
  radio.finishTransmit();
  if (done) { printk("tx done: len=%d ms=%lld\n", len, k_uptime_get() - t0); n_tx++; }
  else      { printk("tx err: timeout after %u ms\n", limit); n_tx_err++; }
  arm_rx();
}

/* ---------- command parsing ---------- */
static int parse_hex(const char* s, uint8_t* out, int max) {
  int n = 0;
  while (*s && n < max) {
    while (*s == ' ') s++;
    if (!isxdigit((unsigned char)s[0]) || !isxdigit((unsigned char)s[1])) break;
    auto hv = [](char c) { return c <= '9' ? c - '0' : (c | 32) - 'a' + 10; };
    out[n++] = (uint8_t)((hv(s[0]) << 4) | hv(s[1]));
    s += 2;
  }
  return n;
}

static bool set_key(Config& c, const char* k, const char* v) {
  if (!strcmp(k, "freq"))  { c.freq_hz = (uint32_t)strtoul(v, nullptr, 10); return c.freq_hz >= 150000000 && c.freq_hz <= 2500000000u; }
  if (!strcmp(k, "sf"))    { c.sf = (uint8_t)atoi(v); return c.sf >= 5 && c.sf <= 12; }
  if (!strcmp(k, "bw"))    { c.bw_hz = (uint32_t)strtoul(v, nullptr, 10); return c.bw_hz >= 7800 && c.bw_hz <= 1000000; }
  if (!strcmp(k, "cr"))    { int code = cr_code_from(v); if (code < 0) return false; c.cr_code = (uint8_t)code; c.cr = code <= 4 ? code + 4 : (code == 5 ? 5 : code == 6 || code == 8 ? 6 : 8); return true; }
  if (!strcmp(k, "pwr"))   { c.pwr = (int8_t)atoi(v); return c.pwr >= -18 && c.pwr <= 22; }  // clamped per band in apply_config()
  if (!strcmp(k, "boost")) { c.boost = (uint8_t)atoi(v); return c.boost <= 7; }
  if (!strcmp(k, "sync"))  { c.sync = (uint8_t)strtoul(v, nullptr, 16); return true; }
  if (!strcmp(k, "pre"))   { c.pre = (uint16_t)atoi(v); return c.pre >= 4 && c.pre <= 1000; }
  if (!strcmp(k, "sd")) {
    c.nside = 0;
    if (!strcmp(v, "none") || !*v) return true;
    // strtok_r: the caller is itself iterating with strtok over the command line, and
    // a nested strtok() resets it - every key after "sd=" used to be silently dropped.
    char tmp[32]; strncpy(tmp, v, sizeof(tmp) - 1); tmp[sizeof(tmp) - 1] = 0;
    char* save = nullptr;
    for (char* t = strtok_r(tmp, ",", &save); t; t = strtok_r(nullptr, ",", &save)) {
      if (c.nside >= 3) return false;
      int sf = atoi(t);
      if (sf < 5 || sf > 12) return false;
      c.side[c.nside++] = (uint8_t)sf;
    }
    return true;
  }
  return false;
}

static void handle_line(char* line) {
  while (*line == ' ') line++;
  size_t n = strlen(line);
  while (n && (line[n - 1] == ' ' || line[n - 1] == '\r')) line[--n] = 0;
  if (!n) return;

  if (!strncmp(line, "tx ", 3)) {
    static uint8_t pkt[RADIOLIB_LR2021_MAX_PACKET_LENGTH];
    int len = parse_hex(line + 3, pkt, sizeof(pkt));
    if (len <= 0) { printk("err tx: no hex payload\n"); return; }
    do_tx(pkt, len);
    return;
  }
  if (!strncmp(line, "set ", 4) || !strcmp(line, "set")) {
    Config nc = cfg;
    char* p = line + 3;
    char* save = nullptr;
    for (char* tok = strtok_r(p, " ", &save); tok; tok = strtok_r(nullptr, " ", &save)) {
      char* eq = strchr(tok, '=');
      if (!eq) { printk("err set: expected key=value, got '%s'\n", tok); return; }
      *eq = 0;
      if (!set_key(nc, tok, eq + 1)) { printk("err set: bad %s=%s\n", tok, eq + 1); return; }
    }
    for (int i = 0; i < nc.nside; i++) {
      if (nc.side[i] <= nc.sf || nc.side[i] > nc.sf + 4) {
        printk("err set: side detector sf%u must be > sf%u and <= sf%u\n", nc.side[i], nc.sf, nc.sf + 4);
        return;
      }
    }
    Config old = cfg;
    cfg = nc;
    if (apply_config() != RADIOLIB_ERR_NONE) {
      cfg = old;
      apply_config();
      printk("err set: chip rejected the configuration, previous one restored\n");
      return;
    }
    print_cfg("ok ");
    return;
  }
  if (!strcmp(line, "status")) {
    uint8_t maj = 0, min = 0;
    radio.getVersion(&maj, &min);
    print_cfg("");
    bool pram = false; uint16_t pv = 0; radio.checkPramLoaded(&pram); if (pram) radio.getPramVersion(&pv);
    printk("status: rx=%u rx_err=%u tx=%u tx_err=%u last_rx_ms=%lld uptime_ms=%lld chip=%u.%u pram=%u irq=%08x\n",
           n_rx, n_rx_err, n_tx, n_tx_err, last_rx_ms, k_uptime_get(), maj, min, pram ? pv : 0, radio.getIrqFlags());
    return;
  }
  if (!strncmp(line, "rng", 3)) {
    // rng sub [addr]          become a ranging subordinate (until "rng off")
    // rng req [addr] [count]  manager: run <count> exchanges with subordinate <addr>
    // rng delay <n>           override the Tx->Rx delay indicator (0 = Semtech table)
    // rng off                 back to LoRa
    char* tok = strtok(line + 3, " ");
    if (!tok) { printk("err rng: sub|req|delay|off\n"); return; }
    if (!strcmp(tok, "off")) { rng_off(); printk("ok rng off\n"); return; }
    if (!strcmp(tok, "delay")) { char* v = strtok(nullptr, " "); rng_user_delay = v ? strtoul(v, nullptr, 10) : 0; printk("ok rng delay=%u (table %u)\n", rng_user_delay, rng_delay_indicator()); return; }
    if (!strcmp(tok, "sub")) {   // rng sub [addr] [window_ms]: window > 0 returns to LoRa by itself
      char* a = strtok(nullptr, " "); uint32_t addr = a ? strtoul(a, nullptr, 16) : rng_addr;
      char* w = strtok(nullptr, " "); uint32_t window = w ? strtoul(w, nullptr, 10) : 0;
      if (rng_setup(true, addr) != RADIOLIB_ERR_NONE) return;
      int16_t st = rng_arm_subordinate();
      rng_sub_until = window ? k_uptime_get() + window : 0;
      printk("%s rng sub addr=%08x freq=%u bw=%u sf=%u delay=%u window=%u\n", st == RADIOLIB_ERR_NONE ? "ok" : "err", addr, cfg.freq_hz, cfg.bw_hz, cfg.sf, rng_delay_indicator(), window);
      return;
    }
    if (!strcmp(tok, "req")) {
      char* a = strtok(nullptr, " "); uint32_t addr = a ? strtoul(a, nullptr, 16) : rng_addr;
      char* c = strtok(nullptr, " "); int count = c ? atoi(c) : 10; if (count < 1) count = 1; if (count > 200) count = 200;
      if (rng_setup(false, addr) != RADIOLIB_ERR_NONE) return;
      printk("ok rng req addr=%08x freq=%u bw=%u sf=%u delay=%u count=%d\n", addr, cfg.freq_hz, cfg.bw_hz, cfg.sf, rng_delay_indicator(), count);
      static int32_t results[200]; int n = 0, timeouts = 0;
      for (int i = 0; i < count; i++) {
        int32_t raw = 0; uint8_t rssi = 0; uint32_t irq = 0;
        if (rng_exchange(&raw, &rssi, &irq)) {
          results[n++] = raw;
          printk("rng result: raw=%d dist=%.2f m rssi=%u\n", raw, (double)rng_metres(raw), rssi);
        } else {
          timeouts++;
          printk("rng result: no response (irq=%08x)\n", irq);
        }
        k_msleep(50);
      }
      if (n) {  // median
        for (int i = 1; i < n; i++) { int32_t v = results[i]; int j = i - 1; while (j >= 0 && results[j] > v) { results[j + 1] = results[j]; j--; } results[j + 1] = v; }
        int32_t med = results[n / 2];
        printk("rng summary: %d/%d valid, median raw=%d dist=%.2f m, min %.2f max %.2f\n", n, count, med, (double)rng_metres(med), (double)rng_metres(results[0]), (double)rng_metres(results[n - 1]));
      } else {
        printk("rng summary: 0/%d valid\n", count);
      }
      rng_off();
      return;
    }
    printk("err rng: unknown '%s'\n", tok);
    return;
  }
  if (!strncmp(line, "wmbus", 5)) {
    // wmbus <t|c|c2|s|r|n48> [freq_hz]   receive wireless M-Bus meters; wmbus off -> LoRa
    char* tok = strtok(line + 5, " ");
    if (!tok || !strcmp(tok, "off")) { wmbus_stop(); printk("ok wmbus off (%u telegrams)\n", n_wmbus); return; }
    struct { const char* n; uint8_t mode; uint32_t f; } modes[] = {
      {"t", 0x03, 868950000}, {"c", 0x05, 868950000}, {"c2", 0x07, 868950000}, {"s", 0x00, 868300000},
      {"r", 0x04, 868330000}, {"n48", 0x08, 169406250}, {"t1", 0x01, 868950000}, {"t2rx", 0x02, 868950000},
    };
    for (auto& m : modes) if (!strcmp(tok, m.n)) {
      char* f = strtok(nullptr, " "); uint32_t freq = f ? strtoul(f, nullptr, 10) : m.f;
      if (wmbus_start(m.mode, freq) == RADIOLIB_ERR_NONE) printk("ok wmbus mode=%s (chip %u) freq=%u boost=%u\n", m.n, m.mode, freq, cfg.boost);
      return;
    }
    printk("err wmbus: mode t|c|c2|s|r|n48|t1|t2rx|off\n");
    return;
  }
  if (!strncmp(line, "pa ", 3)) {   // raw SetPaConfig probe: pa <sel> <lfmode> <lfduty> <slices> <hfduty> [txpower]
    int a[6] = {0, 0, 6, 7, 16, 0}; int n = 0;
    for (char* t = strtok(line + 3, " "); t && n < 6; t = strtok(nullptr, " ")) a[n++] = atoi(t);
    radio.standby();
    int16_t st = radio.setPaConfig(a[0], a[1], a[2], a[3], a[4]);
    int16_t st2 = radio.setTxParams((int8_t)a[5], RADIOLIB_LRXXXX_PA_RAMP_48U);
    uint16_t errs = 0; radio.getErrors(&errs);
    printk("pa sel=%d mode=%d lfduty=%d slices=%d hfduty=%d -> setPaConfig %d, setTxParams(%d) %d, errors=%04x\n",
           a[0], a[1], a[2], a[3], a[4], st, a[5], st2, errs);
    return;
  }
  if (!strcmp(line, "rearm")) { printk("%s\n", arm_rx() == RADIOLIB_ERR_NONE ? "ok" : "err rearm"); return; }
  if (!strcmp(line, "reset")) { printk("ok rebooting\n"); k_msleep(50); sys_reboot(SYS_REBOOT_COLD); }
  if (!strcmp(line, "help") || !strcmp(line, "?")) {
    printk("ok commands: tx <hex> | set freq= sf= bw= cr=<5..8|5li|6li|8li|6cc|8cc> sd=<sf,..|none> pwr= boost= sync= pre= | rng sub|req|delay|off | wmbus <mode>|off | status | rearm | reset\n");
    return;
  }
  printk("err unknown command '%s'\n", line);
}

int main(void) {
  printk("\n%s %s booting\n", FW_NAME, FW_VERSION);

  if (device_is_ready(uart_dev)) {
    uart_irq_callback_user_data_set(uart_dev, uart_isr, nullptr);
    uart_irq_rx_enable(uart_dev);
  }

  radio.irqDioNum = LR2021_IRQ_DIO;
#ifndef LR2021_STOCK_PA_TABLE               // -DLR2021_STOCK_PA_TABLE builds the comparison firmware
  radio.setPaTable(wio_pa_lf, false);   // before begin(): setOutputPower() reads it
#endif
  int16_t st = RADIOLIB_ERR_NONE;
  for (int attempt = 0; attempt < 5; attempt++) {
    // tcxoVoltage 0: the Wio-LR2021 runs on a 32 MHz crystal, skip SetTcxoMode.
    st = radio.begin(cfg.freq_hz / 1000000.0f, cfg.bw_hz / 1000.0f, cfg.sf, cfg.cr, cfg.sync,
                     cfg.pwr, cfg.pre, 0.0f);
    if (st == RADIOLIB_ERR_NONE) break;
    printk("err begin %d (attempt %d), resetting chip\n", st, attempt + 1);
    radio.reset();
    k_msleep(100);
  }
  if (st != RADIOLIB_ERR_NONE) {
    printk("fatal: LR2021 init failed (%d) - check the shield\n", st);
    while (1) k_msleep(1000);
  }
#ifndef LR2021_NO_PRAM
  {
    // Semtech's firmware patch (datasheet 22.3, "highly recommended"); it does not
    // survive the reset begin() just did, and RadioLib never loads it.
    uint16_t pram_ver = 0;
    int16_t ps = lr2021_load_pram(radio, &pram_ver);
    if (ps == RADIOLIB_ERR_NONE) printk("pram loaded, version %u\n", pram_ver);
    else printk("err pram load %d (continuing without it)\n", ps);
  }
#endif
  radio.setPacketReceivedAction(on_irq);
  if (apply_config() != RADIOLIB_ERR_NONE) printk("err initial config\n");

  uint8_t maj = 0, min = 0;
  radio.getVersion(&maj, &min);
  printk("%s %s ready chip=%u.%u\n", FW_NAME, FW_VERSION, maj, min);
  print_cfg("");
#ifdef RNG_SUB_AT_BOOT
  // Bench build: come up as a ranging subordinate on 2450 MHz / 500 kHz / SF8 so
  // the board only needs USB power, not a host (the kit is moved around for tests).
  cfg.freq_hz = 2450000000u; cfg.bw_hz = 500000; cfg.sf = 8; cfg.cr = 5; cfg.nside = 0; cfg.pwr = 10;
  apply_config();
  rng_user_delay = RNG_SUB_AT_BOOT;
  if (rng_setup(true, rng_addr) == RADIOLIB_ERR_NONE) {
    rng_arm_subordinate();
    printk("ok rng sub at boot addr=%08x delay=%u\n", rng_addr, rng_delay_indicator());
  }
#endif

  static char line[1200];
  size_t pos = 0;
  int64_t last_poll = 0;
  while (1) {
    int64_t now = k_uptime_get();
    if (rng_sub_until && now >= rng_sub_until) { rng_off(); printk("rng sub: window over, back to LoRa\n"); }
    if (irq_flag || now - last_poll >= 20) {   // IRQ edge, plus a poll in case one was missed
      irq_flag = false;
      last_poll = now;
      service_radio();
    }
    uint8_t c;
    while (ring_buf_get(&uart_rb, &c, 1) == 1) {
      if (c == '\n' || c == '\r') {
        line[pos] = 0;
        if (pos) handle_line(line);
        pos = 0;
      } else if (pos < sizeof(line) - 1) {
        line[pos++] = (char)c;
      } else {
        pos = 0;   // overlong line: discard
      }
    }
    k_msleep(1);
  }
  return 0;
}
