#include "SerialRadio.h"
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

static speed_t baud_const(int baud) {
  switch (baud) {
    case 9600: return B9600;
    case 57600: return B57600;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return B115200;
  }
}

void SerialRadio::begin() {
  if (_running) return;
  _running = true;
  _reader = std::thread(&SerialRadio::readerLoop, this);
}

void SerialRadio::stop() {
  if (!_running.exchange(false)) return;
  int fd = _fd.exchange(-1);
  if (fd >= 0) close(fd);            // wakes the reader's poll()
  if (_reader.joinable() && _reader.get_id() != std::this_thread::get_id())
    _reader.join();
}

bool SerialRadio::openPort() {
  std::string dev;
  int baud;
  { std::lock_guard<std::mutex> lk(_cfg_mtx); dev = _cfg.device; baud = _cfg.baud; }
  int fd = open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) { perror(("[serial] open " + dev).c_str()); return false; }
  termios t{};
  if (tcgetattr(fd, &t) != 0) { perror("[serial] tcgetattr"); close(fd); return false; }
  cfmakeraw(&t);
  t.c_cflag |= CLOCAL | CREAD;
  t.c_cflag &= ~CRTSCTS;
  t.c_cc[VMIN] = 1; t.c_cc[VTIME] = 0;
  cfsetispeed(&t, baud_const(baud));
  cfsetospeed(&t, baud_const(baud));
  if (tcsetattr(fd, TCSANOW, &t) != 0) { perror("[serial] tcsetattr"); close(fd); return false; }
  tcflush(fd, TCIOFLUSH);
  fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) & ~O_NONBLOCK);
  _fd = fd;
  fprintf(stderr, "[serial] %s open at %d baud\n", dev.c_str(), baud);
  return true;
}

// Keep the port open for the process lifetime: reopen after a USB glitch
// (the bridge re-enumerates) and push our configuration again, since the
// modem may have rebooted meanwhile.
void SerialRadio::readerLoop() {
  char buf[4096];
  char line[2048];
  size_t pos = 0;
  while (_running) {
    if (_fd < 0) {
      if (!openPort()) {
        for (int i = 0; i < 20 && _running; i++) usleep(100 * 1000);   // 2 s
        continue;
      }
      pos = 0;
      sendConfig();
    }
    int fd = _fd;
    pollfd p{fd, POLLIN, 0};
    int r = poll(&p, 1, 1000);
    if (r < 0) { if (errno == EINTR) continue; }
    if (r <= 0) continue;
    ssize_t n = read(fd, buf, sizeof(buf));
    if (n <= 0) {
      if (n < 0 && errno == EAGAIN) continue;
      fprintf(stderr, "[serial] port lost (%s), reopening\n", n == 0 ? "EOF" : strerror(errno));
      int old = _fd.exchange(-1);
      if (old >= 0) close(old);
      { std::lock_guard<std::mutex> lk(_tx_mtx); _tx_result = -1; }
      _tx_cv.notify_all();
      continue;
    }
    for (ssize_t i = 0; i < n; i++) {
      char c = buf[i];
      if (c == '\n' || c == '\r') {
        line[pos] = 0;
        if (pos) handleLine(line);
        pos = 0;
      } else if (pos < sizeof(line) - 1) {
        line[pos++] = c;
      } else {
        pos = 0;
      }
    }
  }
}

// Firmware line contract: "rx cfg: ... snr=<x> rssi=<y> ..." then "rx ok: <hex>";
// "tx done: ..." / "tx err: ..."; everything else is diagnostics.
void SerialRadio::handleLine(char* line) {
  static float pending_snr = 0, pending_rssi = -120;
  static bool has_cfg = false;
  if (!strncmp(line, "rx cfg:", 7)) {
    const char* p = strstr(line, "snr=");
    pending_snr = p ? strtof(p + 4, nullptr) : 0.0f;
    p = strstr(line, "rssi=");
    pending_rssi = p ? strtof(p + 5, nullptr) : -120.0f;
    has_cfg = true;
    fprintf(stderr, "%s\n", line);      // packet log: the same lines lora_rx prints, so
  } else if (!strncmp(line, "rx ok: ", 7)) {
    fprintf(stderr, "%s\n", line);      // the decoder's `stream` command can replay lc.log
    std::vector<uint8_t> pkt;
    for (const char* h = line + 7; h[0] && h[1]; h += 2) {
      auto hex = [](char c) { return c <= '9' ? c - '0' : (c | 32) - 'a' + 10; };
      if (!isxdigit((unsigned char)h[0]) || !isxdigit((unsigned char)h[1])) break;
      pkt.push_back((hex(h[0]) << 4) | hex(h[1]));
    }
    if (!pkt.empty()) {
      std::lock_guard<std::mutex> lk(_mtx);
      _rx.push_back(RxPacket{std::move(pkt), has_cfg ? pending_snr : 0.0f,
                             has_cfg ? pending_rssi : -120.0f});
      _n_recv++;
      if (_rx.size() > 256) _rx.pop_front();
    }
    has_cfg = false;
  } else if (!strncmp(line, "rx err", 6)) {
    _n_err++;
  } else if (!strncmp(line, "rng summary", 11) || !strncmp(line, "ok rng sub", 10) || !strncmp(line, "err rng", 7)) {
    { std::lock_guard<std::mutex> lk(_rng_mtx); _rng_line = line; }
    _rng_cv.notify_all();
    fprintf(stderr, "[serial] %s\n", line);
  } else if (!strncmp(line, "tx done", 7) || !strncmp(line, "tx err", 6)) {
    { std::lock_guard<std::mutex> lk(_tx_mtx); _tx_result = line[3] == 'd' ? 1 : -1; }
    _tx_cv.notify_all();
    fprintf(stderr, "[serial] %s\n", line);   // every transmit is worth a log line
  } else {
    fprintf(stderr, "[serial] %s\n", line);
  }
}

bool SerialRadio::sendLine(const std::string& s) {
  int fd = _fd;
  if (fd < 0) return false;
  std::lock_guard<std::mutex> lk(_wr_mtx);
  std::string out = s + "\n";
  size_t off = 0;
  while (off < out.size()) {
    ssize_t n = write(fd, out.data() + off, out.size() - off);
    if (n < 0) { if (errno == EINTR || errno == EAGAIN) continue; return false; }
    off += (size_t)n;
  }
  return true;
}

std::string SerialRadio::configLine() const {
  // Side detectors must all be above the primary SF and within +4 of it
  // (chip rule); drop the ones the current SF makes invalid.
  std::string sd;
  {
    std::string tmp = _cfg.side_sfs;
    for (char& c : tmp) if (c == ',') c = ' ';
    char* save = nullptr;
    char* buf = tmp.data();
    for (char* t = strtok_r(buf, " ", &save); t; t = strtok_r(nullptr, " ", &save)) {
      int s = atoi(t);
      if (s > _cfg.sf && s <= _cfg.sf + 4 && s <= 12) sd += (sd.empty() ? "" : ",") + std::to_string(s);
    }
  }
  // Preamble as upstream MeshCore transmits it (RadioLibWrapper::preambleLengthForSF):
  // 32 symbols up to SF8, 16 above. Duty-cycled receivers rely on the long one.
  char line[160];
  snprintf(line, sizeof(line), "set freq=%u bw=%u sf=%d cr=%d sd=%s pwr=%d boost=%d pre=%d",
           _cfg.freq, _cfg.bw, _cfg.sf, _cfg.cr, sd.empty() ? "none" : sd.c_str(),
           _cfg.tx_power, _cfg.rx_boost, _cfg.sf <= 8 ? 32 : 16);
  return line;
}

void SerialRadio::sendConfig() {
  std::string line;
  { std::lock_guard<std::mutex> lk(_cfg_mtx); line = configLine(); }
  fprintf(stderr, "[serial] -> %s\n", line.c_str());
  sendLine(line);
}

int SerialRadio::recvRaw(uint8_t* bytes, int sz) {
  std::lock_guard<std::mutex> lk(_mtx);
  if (_rx.empty()) return 0;
  auto pkt = std::move(_rx.front());
  _rx.pop_front();
  _last_snr = pkt.snr;                // published with its packet (see SdrRadio)
  _last_rssi = pkt.rssi;
  int n = (int)std::min<size_t>(pkt.bytes.size(), (size_t)sz);
  memcpy(bytes, pkt.bytes.data(), n);
  return n;
}

bool SerialRadio::startSendRaw(const uint8_t* bytes, int len) {
  if (len <= 0 || len > 255) return false;
  std::string line = "tx ";
  char hx[3];
  for (int i = 0; i < len; i++) { snprintf(hx, sizeof(hx), "%02x", bytes[i]); line += hx; }
  std::unique_lock<std::mutex> lk(_tx_mtx);
  _tx_result = 0;
  fprintf(stderr, "%s\n", line.c_str());   // packet log, outbound side
  if (!sendLine(line)) return false;
  auto limit = std::chrono::milliseconds(getEstAirtimeFor(len) * 2 + 3000);
  bool ok = _tx_cv.wait_for(lk, limit, [&] { return _tx_result != 0; }) && _tx_result == 1;
  if (!ok) fprintf(stderr, "[serial] tx of %d bytes %s\n", len, _tx_result == 0 ? "timed out" : "failed");
  if (ok) _n_sent++;
  return ok;
}

// Semtech airtime, explicit header + CRC, 16-symbol preamble (the firmware's).
uint32_t SerialRadio::getEstAirtimeFor(int len_bytes) {
  int sf, cr; uint32_t bw;
  { std::lock_guard<std::mutex> lk(_cfg_mtx); sf = _cfg.sf; bw = _cfg.bw; cr = _cfg.cr; }
  double tsym = (double)(1u << sf) / bw;
  double nsym = 16 + 4.25 + 8 + std::max(0.0,
      std::ceil((8.0 * len_bytes - 4.0 * sf + 44) / (4.0 * sf)) * cr);
  return (uint32_t)(nsym * tsym * 1000.0);
}

float SerialRadio::packetScore(float snr, int packet_len) {
  // Same 0..1 estimate as SdrRadio (mirrors RadioLibWrapper::packetScoreInt).
  static const float snr_threshold[] = { -7.5f, -10.0f, -12.5f, -15.0f, -17.5f, -20.0f };
  if (!(snr == snr)) return 0.0f;
  int sf;
  { std::lock_guard<std::mutex> lk(_cfg_mtx); sf = _cfg.sf; }
  if (sf < 7 || sf > 12) return 0.0f;
  float thresh = snr_threshold[sf - 7];
  if (snr < thresh) return 0.0f;
  double v = ((snr - thresh) / 10.0) * (1.0 - (packet_len / 256.0));
  return (float)(v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v));
}

void SerialRadio::setParams(float freq_mhz, float bw_khz, uint8_t sf, uint8_t cr) {
  {
    std::lock_guard<std::mutex> lk(_cfg_mtx);
    _cfg.freq = (uint32_t)(llround((double)freq_mhz * 1000.0) * 1000);   // kHz-exact, see SdrRadio
    _cfg.bw = (uint32_t)llround((double)bw_khz * 1000.0);
    _cfg.sf = sf;
    _cfg.cr = cr >= 5 ? cr : cr + 4;
  }
  sendConfig();
}

void SerialRadio::setTxPower(uint8_t dbm) {
  int p = dbm > 22 ? 22 : (int)dbm;
  { std::lock_guard<std::mutex> lk(_cfg_mtx); _cfg.tx_power = p; }
  sendLine("set pwr=" + std::to_string(p));
}

void SerialRadio::setRxBoostedGainMode(bool state) {
  { std::lock_guard<std::mutex> lk(_cfg_mtx); _cfg.rx_boost = state ? 7 : 0; }
  sendLine(std::string("set boost=") + (state ? "7" : "0"));
}

bool SerialRadio::getRxBoostedGainMode() const {
  std::lock_guard<std::mutex> lk(_cfg_mtx);
  return _cfg.rx_boost != 0;
}

// ---- LR2021 time-of-flight ranging through the modem's "rng" commands ----

bool SerialRadio::waitRangingLine(const char* prefix, int timeout_ms, std::string& out) {
  std::unique_lock<std::mutex> lk(_rng_mtx);
  bool ok = _rng_cv.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                             [&] { return !_rng_line.empty() && (_rng_line.rfind(prefix, 0) == 0 || _rng_line.rfind("err rng", 0) == 0); });
  out = _rng_line;
  return ok && out.rfind(prefix, 0) == 0;
}

static std::string rangingSetLine(const RangingRequest& req) {
  char line[128];
  snprintf(line, sizeof(line), "set freq=%u bw=%u sf=%u cr=5 sd=none", req.freq_hz, req.bw_hz, req.sf);
  return line;
}

bool SerialRadio::rangeSubordinate(const RangingRequest& req, uint32_t my_addr) {
  if (_fd < 0) return false;
  uint32_t window = rangingWindowMs(req.count);
  { std::lock_guard<std::mutex> lk(_rng_mtx); _rng_line.clear(); }
  sendLine(rangingSetLine(req));
  sendLine("rng delay " + std::to_string(req.delay));
  char cmd[64]; snprintf(cmd, sizeof(cmd), "rng sub %08x %u", my_addr, window);
  sendLine(cmd);
  std::string ack;
  bool ok = waitRangingLine("ok rng sub", 2000, ack);
  fprintf(stderr, "[serial] ranging subordinate %s for %u ms (addr %08x)\n", ok ? "armed" : "FAILED", window, my_addr);
  if (ok) usleep((window + 200) * 1000);   // the modem returns to LoRa by itself when the window ends
  sendConfig();                             // and we put our own configuration back
  return ok;
}

bool SerialRadio::rangeManager(const RangingRequest& req, uint32_t peer_addr, RangingResult& out) {
  out = RangingResult();
  out.count = req.count;
  if (_fd < 0) { out.status = 2; return false; }
  { std::lock_guard<std::mutex> lk(_rng_mtx); _rng_line.clear(); }
  sendLine(rangingSetLine(req));
  sendLine("rng delay " + std::to_string(req.delay));
  char cmd[64]; snprintf(cmd, sizeof(cmd), "rng req %08x %u", peer_addr, req.count);
  sendLine(cmd);
  std::string sum;
  bool ok = waitRangingLine("rng summary", (int)req.count * 1700 + 4000, sum);
  sendConfig();
  if (!ok) { out.status = 1; return false; }
  // "rng summary: N/M valid, median raw=R dist=D m, min A max B" or "rng summary: 0/M valid"
  int valid = 0, total = 0; float med = 0, mn = 0, mx = 0; int raw = 0;
  if (sscanf(sum.c_str(), "rng summary: %d/%d valid, median raw=%d dist=%f m, min %f max %f", &valid, &total, &raw, &med, &mn, &mx) >= 4 && valid > 0) {
    out.status = 0; out.valid = (uint8_t)valid;
    out.median_cm = (int32_t)lroundf(med * 100); out.min_cm = (int32_t)lroundf(mn * 100); out.max_cm = (int32_t)lroundf(mx * 100);
    return true;
  }
  out.status = 1;
  return false;
}
