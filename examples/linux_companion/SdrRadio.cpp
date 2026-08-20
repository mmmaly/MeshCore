#include "SdrRadio.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#ifdef __linux__
#include <sys/prctl.h>
#endif

// Subprocess plumbing proven in meshcore-open-sdr/sdr-node; reused verbatim.
static pid_t spawn_pipe(const std::vector<std::string>& argv, int* out_fd) {
  int fds[2];
  // O_CLOEXEC so lora_rx inherits nothing of ours but its own stdout: it
  // outlives many client connections, and a stray copy of a client socket
  // keeps that connection half-open long after the daemon has closed it.
  // (dup2 below clears the flag on the descriptor the child actually needs.)
#ifdef __linux__
  if (pipe2(fds, O_CLOEXEC) != 0) return -1;
#else
  if (pipe(fds) != 0) return -1;
  fcntl(fds[0], F_SETFD, FD_CLOEXEC);
  fcntl(fds[1], F_SETFD, FD_CLOEXEC);
#endif
  pid_t pid = fork();
  if (pid < 0) { close(fds[0]); close(fds[1]); return -1; }
  if (pid == 0) {
    close(fds[0]); dup2(fds[1], STDOUT_FILENO); close(fds[1]);
#ifdef __linux__
    prctl(PR_SET_PDEATHSIG, SIGTERM);
    if (getppid() == 1) _exit(0);
#endif
    std::vector<char*> a;
    for (auto& s : argv) a.push_back(const_cast<char*>(s.c_str()));
    a.push_back(nullptr);
    execvp(a[0], a.data());
    perror("execvp"); _exit(127);
  }
  close(fds[1]); *out_fd = fds[0];
  return pid;
}

void SdrRadio::begin() {
  if (_running) return;
  _running = true;
  _reader = std::thread(&SdrRadio::superviseLoop, this);
  _watchdog = std::thread(&SdrRadio::watchdogLoop, this);
}

void SdrRadio::stop() {
  if (!_running.exchange(false)) return;
  // Only signal; the supervisor thread owns the pid and reaps it. Clearing
  // _rx_pid here would let a later kill() land on a recycled pid.
  pid_t pid = _rx_pid;
  if (pid > 0) kill(pid, SIGTERM);
  // Never join from the reader thread itself (a signal can land there).
  if (_reader.joinable() && _reader.get_id() != std::this_thread::get_id())
    _reader.join();
  if (_watchdog.joinable() && _watchdog.get_id() != std::this_thread::get_id())
    _watchdog.join();
}

// A deaf child cannot be told apart from a quiet channel from the inside, so
// the distinction is made the only way it can be: by how long the silence
// lasts. Same cross-thread kill discipline as setParams - signal the pid,
// let the supervisor reap and respawn.
void SdrRadio::watchdogLoop() {
  while (_running) {
    for (int i = 0; i < 100 && _running; i++) usleep(100 * 1000);   // 10 s
    int limit;
    { std::lock_guard<std::mutex> lk(_cfg_mtx);
      limit = _proven ? _cfg.rx_watchdog_s : _cfg.rx_probation_s; }
    if (limit <= 0) continue;
    time_t last = _last_rx;
    pid_t pid = _rx_pid;
    if (pid > 0 && last > 0 && time(nullptr) - last > limit) {
      fprintf(stderr, "[sdr] no packet in %d s%s - restarting rx (deaf tuner?)\n",
              limit, _proven ? "" : " (probation)");
      _last_rx = time(nullptr);   // one kill per silent interval, not one per tick
      kill(pid, SIGTERM);
    }
  }
}

// Reads _cfg from the supervisor thread while the mesh thread may be in
// setParams(), so the whole read happens under the config lock - the strings
// in particular cannot be copied safely while being reassigned.
std::vector<std::string> SdrRadio::rxArgv() const {
  std::lock_guard<std::mutex> lk(_cfg_mtx);
  std::vector<std::string> argv = {_cfg.rx_binary};
  if (!_cfg.rx_channels.empty()) { argv.push_back("-C"); argv.push_back(_cfg.rx_channels); }
  if (!_cfg.rx_sfs.empty())      { argv.push_back("-S"); argv.push_back(_cfg.rx_sfs); }
  argv.push_back("-b"); argv.push_back(std::to_string(_cfg.bw));
  argv.push_back("-p"); argv.push_back(std::to_string(_cfg.rx_ppm));
  if (_cfg.rx_agc) { argv.push_back("-G"); argv.push_back("-T"); }
  if (!_cfg.rx_device.empty()) { argv.push_back("-d"); argv.push_back(_cfg.rx_device); }
  return argv;
}

// Keep a receiver alive for the process lifetime: respawn when lora_rx exits
// (USB glitch, crash, or a deliberate kill after setParams retunes it).
void SdrRadio::superviseLoop() {
  while (_running) {
    int fd = -1;
    pid_t pid = spawn_pipe(rxArgv(), &fd);
    _rx_pid = pid;
    _last_rx = time(nullptr);   // the watchdog clock starts at spawn
    _proven = false;            // every open re-rolls the tuner
    if (pid > 0) {
      _rx_fd = fd;
      readPipe(fd);
      int st = 0; waitpid(pid, &st, 0);
      _rx_pid = -1;      // reaped: no one may signal this pid again
      _rx_fd = -1;
    } else {
      fprintf(stderr, "[sdr] lora_rx spawn failed\n");
    }
    if (_running) {
      for (int i = 0; i < 50 && _running; i++) usleep(100 * 1000);   // 5 s
    }
  }
}

// lora_rx stdout contract: "rx cfg: ... snr=<x> ..." then "rx ok: <hex>".
void SdrRadio::readPipe(int fd) {
  FILE* f = fdopen(fd, "r");
  if (!f) { close(fd); return; }
  char line[16384];
  float pending_snr = 0;
  bool has_snr = false;
  while (_running && fgets(line, sizeof(line), f)) {
    if (!strncmp(line, "rx cfg:", 7)) {
      const char* p = strstr(line, "snr=");
      if (p) { pending_snr = strtof(p + 4, nullptr); has_snr = true; }
    } else if (!strncmp(line, "rx ok: ", 7)) {
      std::vector<uint8_t> pkt;
      for (const char* h = line + 7; h[0] && h[1] && h[0] != '\n'; h += 2) {
        auto hex = [](char c) { return c <= '9' ? c - '0' : (c | 32) - 'a' + 10; };
        if (!isxdigit((unsigned char)h[0]) || !isxdigit((unsigned char)h[1])) break;
        pkt.push_back((hex(h[0]) << 4) | hex(h[1]));
      }
      if (!pkt.empty()) {
        std::lock_guard<std::mutex> lk(_mtx);
        _rx.push_back(RxPacket{std::move(pkt), has_snr ? pending_snr : 0.0f});
        _n_recv++;
        _last_rx = time(nullptr);
        _proven = true;
        if (_rx.size() > 256) _rx.pop_front();
        has_snr = false;   // each "rx ok" consumes the cfg line that preceded it
      }
    }
  }
  fclose(f);
}

int SdrRadio::recvRaw(uint8_t* bytes, int sz) {
  std::lock_guard<std::mutex> lk(_mtx);
  if (_rx.empty()) return 0;
  auto pkt = std::move(_rx.front());
  _rx.pop_front();
  // Publish this packet's SNR as we hand it over: Dispatcher calls
  // getLastSNR() straight after recvRaw() and expects the pair to match.
  _last_snr = pkt.snr;
  int n = (int)std::min<size_t>(pkt.bytes.size(), (size_t)sz);
  memcpy(bytes, pkt.bytes.data(), n);
  return n;
}

bool SdrRadio::startSendRaw(const uint8_t* bytes, int len) {
  char hex[1024]; int n = len < 500 ? len : 500;
  for (int i = 0; i < n; i++) sprintf(hex + i * 2, "%02x", bytes[i]);
  std::vector<std::string> argv;
  {
    std::lock_guard<std::mutex> lk(_cfg_mtx);
    argv = {
      _cfg.tx_binary, "-f", std::to_string(_cfg.tx_freq),
      "-S", std::to_string(_cfg.tx_sf), "-b", std::to_string(_cfg.bw),
      "-c", std::to_string(_cfg.tx_cr), "-p", std::to_string(_cfg.tx_ppm),
      "-g", std::to_string(_cfg.tx_vga), "-y", std::to_string(_cfg.tx_duty),
      "-L", std::to_string(_cfg.tx_level)};
    if (_cfg.tx_amp) argv.push_back("-a");
  }
  argv.push_back("-x"); argv.push_back(hex);
  pid_t pid = fork();
  if (pid < 0) return false;
  if (pid == 0) {
    std::vector<char*> a;
    for (auto& s : argv) a.push_back(const_cast<char*>(s.c_str()));
    a.push_back(nullptr);
    execvp(a[0], a.data()); _exit(127);
  }
  int st = 0; waitpid(pid, &st, 0);
  bool ok = WIFEXITED(st) && WEXITSTATUS(st) == 0;
  if (ok) _n_sent++;
  return ok;
}

// Semtech airtime, approximated (ms).
uint32_t SdrRadio::getEstAirtimeFor(int len_bytes) {
  int sf; uint32_t bw; int cr;
  { std::lock_guard<std::mutex> lk(_cfg_mtx); sf = _cfg.tx_sf; bw = _cfg.bw; cr = _cfg.tx_cr; }
  double tsym = (double)(1u << sf) / bw;
  double nsym = 12.25 + 8 + std::max(0.0,
      std::ceil((8.0 * len_bytes - 4.0 * sf + 44) / (4.0 * sf)) * (cr + 4));
  return (uint32_t)(nsym * tsym * 1000.0);
}

// Mirrors RadioLibWrapper::packetScoreInt: a 0..1 estimate of how likely
// this reception was to succeed, which Dispatcher turns into a rebroadcast
// delay. Returning raw SNR (as a first cut did) puts every packet past the
// threshold, so everything is processed immediately and the delay that
// spaces out flood retransmissions never happens.
float SdrRadio::packetScore(float snr, int packet_len) {
  // Approximate SNR floor per SF, from the Semtech datasheets
  static const float snr_threshold[] = { -7.5f, -10.0f, -12.5f, -15.0f, -17.5f, -20.0f };
  if (!(snr == snr)) return 0.0f;    // NaN loses every comparison below, incl. the clamp
  int sf;
  { std::lock_guard<std::mutex> lk(_cfg_mtx); sf = _cfg.tx_sf; }
  if (sf < 7 || sf > 12) return 0.0f;
  float thresh = snr_threshold[sf - 7];
  if (snr < thresh) return 0.0f;               // no realistic chance
  double success = (snr - thresh) / 10.0;
  double collision_penalty = 1.0 - (packet_len / 256.0);
  double v = success * collision_penalty;
  return (float)(v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v));
}

void SdrRadio::setParams(float freq_mhz, float bw_khz, uint8_t sf, uint8_t cr) {
  uint32_t freq, bw;
  {
    std::lock_guard<std::mutex> lk(_cfg_mtx);
    // Round to kHz: prefs.freq is a float in MHz, so 869.618 lands on
    // 869617984 Hz if taken literally - 16 Hz off, and enough to make the
    // channel list disagree with itself.
    _cfg.tx_freq = (uint32_t)(llround((double)freq_mhz * 1000.0) * 1000);
    _cfg.bw = (uint32_t)llround((double)bw_khz * 1000.0);
    _cfg.tx_sf = sf;
    _cfg.tx_cr = cr >= 5 ? cr - 4 : cr;
    // One radio, one channel - matching what the firmware above believes it
    // has. (Listening to several at once is an SDR luxury the mesh stack has
    // no way to express.)
    _cfg.rx_channels = std::to_string(_cfg.tx_freq);
    _cfg.rx_sfs = std::to_string(sf);
    freq = _cfg.tx_freq; bw = _cfg.bw;
  }
  fprintf(stderr, "[sdr] params: %u Hz bw %u sf %u cr %u - restarting rx\n", freq, bw, sf, cr);
  pid_t pid = _rx_pid;
  if (pid > 0) kill(pid, SIGTERM);   // reader loop exits; supervisor restarts it
}

void SdrRadio::setRxBoostedGainMode(bool state) {
  std::lock_guard<std::mutex> lk(_cfg_mtx);
  _cfg.rx_agc = state;
}

bool SdrRadio::getRxBoostedGainMode() const {
  std::lock_guard<std::mutex> lk(_cfg_mtx);
  return _cfg.rx_agc;
}

void SdrRadio::setTxPower(uint8_t dbm) {
  // HackRF gain is set host-side (tx_vga/tx_amp); nothing to do per-packet.
  fprintf(stderr, "[sdr] tx power request %u dBm (host-configured gains)\n", dbm);
}
