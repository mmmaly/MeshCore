#include "SdrRadio.h"
#include <cstdio>
#include <cstring>
#include <cmath>
#include <unistd.h>
#include <signal.h>
#include <sys/wait.h>
#ifdef __linux__
#include <sys/prctl.h>
#endif

// Subprocess plumbing proven in meshcore-open-sdr/sdr-node; reused verbatim.
static pid_t spawn_pipe(const std::vector<std::string>& argv, int* out_fd) {
  int fds[2];
  if (pipe(fds) != 0) return -1;
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
  _reader = std::thread(&SdrRadio::readerLoop, this);
}

void SdrRadio::stop() {
  if (!_running) return;
  _running = false;
  if (_rx_pid > 0) { kill(_rx_pid, SIGTERM); waitpid(_rx_pid, nullptr, 0); _rx_pid = -1; }
  if (_reader.joinable()) _reader.join();
}

// lora_rx stdout contract: "rx cfg: ... snr=<x> ..." then "rx ok: <hex>".
void SdrRadio::readerLoop() {
  std::vector<std::string> argv = {_cfg.rx_binary};
  if (!_cfg.rx_channels.empty()) { argv.push_back("-C"); argv.push_back(_cfg.rx_channels); }
  if (!_cfg.rx_sfs.empty())      { argv.push_back("-S"); argv.push_back(_cfg.rx_sfs); }
  argv.push_back("-b"); argv.push_back(std::to_string(_cfg.bw));
  argv.push_back("-p"); argv.push_back(std::to_string(_cfg.rx_ppm));
  if (_cfg.rx_agc) { argv.push_back("-G"); argv.push_back("-T"); }
  if (!_cfg.rx_device.empty()) { argv.push_back("-d"); argv.push_back(_cfg.rx_device); }

  int fd = -1;
  _rx_pid = spawn_pipe(argv, &fd);
  if (_rx_pid < 0) { fprintf(stderr, "[sdr] lora_rx spawn failed\n"); return; }
  _rx_fd = fd;
  FILE* f = fdopen(fd, "r");
  if (!f) { close(fd); return; }
  char line[16384];
  float pending_snr = 0;
  while (_running && fgets(line, sizeof(line), f)) {
    if (!strncmp(line, "rx cfg:", 7)) {
      const char* p = strstr(line, "snr=");
      if (p) pending_snr = strtof(p + 4, nullptr);
    } else if (!strncmp(line, "rx ok: ", 7)) {
      std::vector<uint8_t> pkt;
      for (const char* h = line + 7; h[0] && h[1] && h[0] != '\n'; h += 2) {
        auto hex = [](char c) { return c <= '9' ? c - '0' : (c | 32) - 'a' + 10; };
        if (!isxdigit((unsigned char)h[0]) || !isxdigit((unsigned char)h[1])) break;
        pkt.push_back((hex(h[0]) << 4) | hex(h[1]));
      }
      if (!pkt.empty()) {
        std::lock_guard<std::mutex> lk(_mtx);
        _last_snr = pending_snr;
        _rx.push_back(std::move(pkt));
        if (_rx.size() > 256) _rx.pop_front();
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
  int n = (int)std::min<size_t>(pkt.size(), (size_t)sz);
  memcpy(bytes, pkt.data(), n);
  return n;
}

bool SdrRadio::startSendRaw(const uint8_t* bytes, int len) {
  char hex[1024]; int n = len < 500 ? len : 500;
  for (int i = 0; i < n; i++) sprintf(hex + i * 2, "%02x", bytes[i]);
  std::vector<std::string> argv = {
    _cfg.tx_binary, "-f", std::to_string(_cfg.tx_freq),
    "-S", std::to_string(_cfg.tx_sf), "-b", std::to_string(_cfg.bw),
    "-c", std::to_string(_cfg.tx_cr), "-p", std::to_string(_cfg.tx_ppm),
    "-g", std::to_string(_cfg.tx_vga), "-y", std::to_string(_cfg.tx_duty)};
  if (_cfg.tx_amp) argv.push_back("-a");
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
  return WIFEXITED(st) && WEXITSTATUS(st) == 0;
}

// Semtech airtime, approximated (ms).
uint32_t SdrRadio::getEstAirtimeFor(int len_bytes) {
  double tsym = (double)(1u << _cfg.tx_sf) / _cfg.bw;
  double nsym = 12.25 + 8 + std::max(0.0,
      std::ceil((8.0 * len_bytes - 4.0 * _cfg.tx_sf + 44) / (4.0 * _cfg.tx_sf)) * (_cfg.tx_cr + 4));
  return (uint32_t)(nsym * tsym * 1000.0);
}

float SdrRadio::packetScore(float snr, int packet_len) {
  return snr;   // simple: higher SNR wins (firmware weights by airtime too)
}
