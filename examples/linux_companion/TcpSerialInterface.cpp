#include "TcpSerialInterface.h"
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>

static const uint8_t FRAME_TO_NODE = 0x3C, FRAME_TO_APP = 0x3E;
static const size_t MAX_PAYLOAD = 172;

bool TcpSerialInterface::start() {
  _srv = socket(AF_INET, SOCK_STREAM, 0);
  int one = 1;
  setsockopt(_srv, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  sockaddr_in a{};
  a.sin_family = AF_INET; a.sin_addr.s_addr = INADDR_ANY; a.sin_port = htons((uint16_t)_port);
  if (bind(_srv, (sockaddr*)&a, sizeof(a)) || listen(_srv, 2)) { perror("bind/listen"); return false; }
  _running = true;
  _accept = std::thread(&TcpSerialInterface::acceptLoop, this);
  fprintf(stderr, "[tcp] listening on port %d\n", _port);
  return true;
}

void TcpSerialInterface::stop() {
  _running = false;
  if (_srv >= 0) { close(_srv); _srv = -1; }
  int c = _client.exchange(-1);
  if (c >= 0) close(c);
  if (_accept.joinable()) _accept.join();
}

void TcpSerialInterface::acceptLoop() {
  while (_running) {
    int fd = accept(_srv, nullptr, nullptr);
    if (fd < 0) continue;
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    int old = _client.exchange(fd);
    if (old >= 0) close(old);       // one client at a time, newest wins
    { std::lock_guard<std::mutex> lk(_mtx); _in.clear(); _frames.clear(); }
    fprintf(stderr, "[tcp] client connected\n");

    uint8_t buf[4096];
    while (_running) {
      ssize_t n = read(fd, buf, sizeof(buf));
      if (n <= 0) break;
      std::lock_guard<std::mutex> lk(_mtx);
      _in.insert(_in.end(), buf, buf + n);
      size_t pos = 0;
      while (_in.size() - pos >= 3) {
        if (_in[pos] != FRAME_TO_NODE) { pos++; continue; }
        size_t len = _in[pos + 1] | ((size_t)_in[pos + 2] << 8);
        if (len > MAX_PAYLOAD) { pos++; continue; }
        if (_in.size() - pos < 3 + len) break;
        _frames.emplace_back(_in.begin() + pos + 3, _in.begin() + pos + 3 + len);
        pos += 3 + len;
      }
      _in.erase(_in.begin(), _in.begin() + pos);
    }
    if (_client == fd) _client = -1;
    close(fd);
    fprintf(stderr, "[tcp] client disconnected\n");
  }
}

size_t TcpSerialInterface::writeFrame(const uint8_t src[], size_t len) {
  int fd = _client;
  if (fd < 0 || len == 0 || len > MAX_PAYLOAD) return 0;
  uint8_t hdr[3] = {FRAME_TO_APP, (uint8_t)(len & 0xFF), (uint8_t)(len >> 8)};
  if (write(fd, hdr, 3) != 3) return 0;
  if (write(fd, src, len) != (ssize_t)len) return 0;
  return len;
}

size_t TcpSerialInterface::checkRecvFrame(uint8_t dest[]) {
  std::lock_guard<std::mutex> lk(_mtx);
  if (_frames.empty()) return 0;
  auto f = std::move(_frames.front());
  _frames.pop_front();
  memcpy(dest, f.data(), f.size());
  return f.size();
}
