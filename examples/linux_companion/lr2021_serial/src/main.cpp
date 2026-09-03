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

#define FW_NAME "lr2021_serial"
#define FW_VERSION "1.0"

#ifndef LR2021_IRQ_DIO
#define LR2021_IRQ_DIO 8          // Wio-LR2021 wires DIO8 to the host IRQ line
#endif

static ZephyrHal hal;
static Module mod(&hal, LR_PIN_NSS, LR_PIN_IRQ, LR_PIN_RESET, LR_PIN_BUSY);
static LR2021 radio(&mod);

struct Config {
  uint32_t freq_hz = 869432000;   // MeshCore CZ community preset
  uint8_t  sf = 7;
  uint32_t bw_hz = 62500;
  uint8_t  cr = 5;                // 4/5 .. 4/8, RadioLib convention
  uint8_t  side[3] = {8, 0, 0};   // side-detector SFs, all > sf, span <= 4
  uint8_t  nside = 1;
  int8_t   pwr = 22;              // dBm; the sub-GHz PA goes to +22
  uint8_t  boost = 7;             // RX boosted gain level (Semtech usp default 7)
  uint8_t  sync = 0x12;           // private network sync word (MeshCore)
  uint16_t pre = 16;              // preamble symbols on TX (MeshCore uses 16)
};
static Config cfg;

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
static void print_cfg(const char* prefix) {
  char sd[16] = "none";
  if (cfg.nside) {
    int p = 0;
    for (int i = 0; i < cfg.nside; i++) p += snprintf(sd + p, sizeof(sd) - p, "%s%u", i ? "," : "", cfg.side[i]);
  }
  printk("%scfg: freq=%u sf=%u bw=%u cr=%u sd=%s pwr=%d boost=%u sync=%02x pre=%u\n",
         prefix, cfg.freq_hz, cfg.sf, cfg.bw_hz, cfg.cr, sd, cfg.pwr, cfg.boost, cfg.sync, cfg.pre);
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
  int16_t st = radio.setFrequency(cfg.freq_hz / 1000000.0f);
  if (st == RADIOLIB_ERR_NONE) st = radio.setBandwidth(cfg.bw_hz / 1000.0f);
  if (st == RADIOLIB_ERR_NONE) st = radio.setSpreadingFactor(cfg.sf);
  if (st == RADIOLIB_ERR_NONE) st = radio.setCodingRate(cfg.cr);
  if (st == RADIOLIB_ERR_NONE) st = radio.setSyncWord(cfg.sync);
  if (st == RADIOLIB_ERR_NONE) st = radio.setPreambleLength(cfg.pre);
  if (st == RADIOLIB_ERR_NONE) st = radio.setOutputPower(cfg.pwr);
  if (st == RADIOLIB_ERR_NONE) st = radio.setRxBoostedGainMode(cfg.boost);
  if (st == RADIOLIB_ERR_NONE) st = radio.setCRC(2);
  if (st == RADIOLIB_ERR_NONE) st = radio.explicitHeader();
  if (st != RADIOLIB_ERR_NONE) { printk("err modulation %d\n", st); return st; }

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

/* ---------- radio service: called on IRQ edge and as a periodic poll ---------- */
static void service_radio() {
  uint32_t irq = radio.getIrqFlags();
  if (!irq) return;

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
      printk("rx cfg: freq=%u sf=%u bw=%u cr=%u snr=%.1f rssi=%.1f det=%u len=%u time=%lld\n",
             cfg.freq_hz, sf_rx, cfg.bw_hz, (unsigned)cr + 4, (double)snr, (double)rssi, det,
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
  if (!strcmp(k, "cr"))    { int cr = atoi(v); if (cr >= 1 && cr <= 4) cr += 4; c.cr = (uint8_t)cr; return cr >= 5 && cr <= 8; }
  if (!strcmp(k, "pwr"))   { c.pwr = (int8_t)atoi(v); return c.pwr >= -9 && c.pwr <= 22; }
  if (!strcmp(k, "boost")) { c.boost = (uint8_t)atoi(v); return c.boost <= 7; }
  if (!strcmp(k, "sync"))  { c.sync = (uint8_t)strtoul(v, nullptr, 16); return true; }
  if (!strcmp(k, "pre"))   { c.pre = (uint16_t)atoi(v); return c.pre >= 4 && c.pre <= 1000; }
  if (!strcmp(k, "sd")) {
    c.nside = 0;
    if (!strcmp(v, "none") || !*v) return true;
    char tmp[32]; strncpy(tmp, v, sizeof(tmp) - 1); tmp[sizeof(tmp) - 1] = 0;
    for (char* t = strtok(tmp, ","); t; t = strtok(nullptr, ",")) {
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
    for (char* tok = strtok(p, " "); tok; tok = strtok(nullptr, " ")) {
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
    printk("status: rx=%u rx_err=%u tx=%u tx_err=%u last_rx_ms=%lld uptime_ms=%lld chip=%u.%u irq=%08x\n",
           n_rx, n_rx_err, n_tx, n_tx_err, last_rx_ms, k_uptime_get(), maj, min, radio.getIrqFlags());
    return;
  }
  if (!strcmp(line, "rearm")) { printk("%s\n", arm_rx() == RADIOLIB_ERR_NONE ? "ok" : "err rearm"); return; }
  if (!strcmp(line, "reset")) { printk("ok rebooting\n"); k_msleep(50); sys_reboot(SYS_REBOOT_COLD); }
  if (!strcmp(line, "help") || !strcmp(line, "?")) {
    printk("ok commands: tx <hex> | set freq= sf= bw= cr= sd=<sf,..|none> pwr= boost= sync= pre= | status | rearm | reset\n");
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
  radio.setPacketReceivedAction(on_irq);
  if (apply_config() != RADIOLIB_ERR_NONE) printk("err initial config\n");

  uint8_t maj = 0, min = 0;
  radio.getVersion(&maj, &min);
  printk("%s %s ready chip=%u.%u\n", FW_NAME, FW_VERSION, maj, min);
  print_cfg("");

  static char line[1200];
  size_t pos = 0;
  int64_t last_poll = 0;
  while (1) {
    int64_t now = k_uptime_get();
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
