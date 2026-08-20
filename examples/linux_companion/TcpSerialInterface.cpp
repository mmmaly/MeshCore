#include "TcpSerialInterface.h"
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>

static const uint8_t FRAME_TO_NODE = 0x3C, FRAME_TO_APP = 0x3E;
// Take the cap from BaseSerialInterface rather than restating it: MyMesh
// really does emit frames of the full MAX_FRAME_SIZE (176, which is 172 plus
// 4 for transport codes), and a lower local limit silently drops them on the
// way out and mis-frames them on the way in.
static const size_t MAX_PAYLOAD = MAX_FRAME_SIZE;

// The app can vanish without sending a FIN (phone leaves Wi-Fi), which used to
// strand the reader in a blocking read() forever. Keepalives turn that into a
// detectable death in ~40 s.
static void configure_client(int fd) {
  int one = 1;
  setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
  setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &one, sizeof(one));
#ifdef TCP_KEEPIDLE
  int idle = 20, intvl = 5, cnt = 4;
  setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
  setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &intvl, sizeof(intvl));
  setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &cnt, sizeof(cnt));
#endif
  // writeFrame runs on the mesh thread; a client that stops reading must not
  // be able to block it indefinitely.
  struct timeval tv { 2, 0 };
  setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
}

bool TcpSerialInterface::start() {
  int type = SOCK_STREAM;
#ifdef SOCK_CLOEXEC
  type |= SOCK_CLOEXEC;    // lora_rx must not inherit our sockets
#endif
  _srv = socket(AF_INET, type, 0);
  if (_srv < 0) { perror("socket"); return false; }
#ifndef SOCK_CLOEXEC
  fcntl(_srv, F_SETFD, FD_CLOEXEC);
#endif
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
  if (!_running.exchange(false)) return;
  // Only the accept thread closes sockets; everyone else shuts them down, so
  // a descriptor can never be closed out from under a concurrent poll/read
  // and later reused for something unrelated.
  if (_srv >= 0) shutdown(_srv, SHUT_RDWR);
  int c = _client;
  if (c >= 0) shutdown(c, SHUT_RDWR);
  if (_accept.joinable() && _accept.get_id() != std::this_thread::get_id())
    _accept.join();
  c = _client.exchange(-1);
  if (c >= 0) close(c);
  if (_srv >= 0) { close(_srv); _srv = -1; }
}

// Wake the accept thread out of poll()/read() on this client and let it do the
// closing. Called from the mesh thread, so it must not close() the fd itself.
void TcpSerialInterface::dropClient(int fd) {
  if (fd >= 0 && _client == fd) shutdown(fd, SHUT_RDWR);
}

// A single thread owns both the listener and the client socket. Polling them
// together is what keeps a live-but-silent client from locking out every new
// connection, which is exactly what happened when accept() could only be
// reached after the previous client's read loop returned.
void TcpSerialInterface::acceptLoop() {
  uint8_t buf[4096];
  while (_running) {
    struct pollfd fds[2];
    fds[0].fd = _srv; fds[0].events = POLLIN; fds[0].revents = 0;
    int client = _client;
    int nfds = 1;
    if (client >= 0) {
      fds[1].fd = client; fds[1].events = POLLIN; fds[1].revents = 0;
      nfds = 2;
    }
    int rc = poll(fds, nfds, 500);
    if (rc < 0) {
      if (errno == EINTR) continue;
      perror("poll");
      break;
    }
    if (rc == 0) continue;

    if (fds[0].revents & POLLIN) {
      int fd;
#if defined(__linux__) && defined(SOCK_CLOEXEC)
      fd = accept4(_srv, nullptr, nullptr, SOCK_CLOEXEC);
#else
      fd = accept(_srv, nullptr, nullptr);
      if (fd >= 0) fcntl(fd, F_SETFD, FD_CLOEXEC);
#endif
      if (fd >= 0) {
        configure_client(fd);
        int old = _client.exchange(fd);
        if (old >= 0) close(old);      // one client at a time, newest wins
        { std::lock_guard<std::mutex> lk(_mtx); _in.clear(); _frames.clear(); }
        fprintf(stderr, "[tcp] client connected\n");
      }
      continue;   // re-poll: `client` below may now be the socket just closed
    }

    if (nfds == 2 && (fds[1].revents & (POLLIN | POLLHUP | POLLERR))) {
      ssize_t n = read(client, buf, sizeof(buf));
      if (n > 0) {
        consume(buf, (size_t)n);
      } else if (n < 0 && (errno == EINTR || errno == EAGAIN)) {
        continue;
      } else {
        int expected = client;
        if (_client.compare_exchange_strong(expected, -1)) {
          close(client);
          fprintf(stderr, "[tcp] client disconnected\n");
        }
      }
    }
  }
}

void TcpSerialInterface::consume(const uint8_t* data, size_t n) {
  std::lock_guard<std::mutex> lk(_mtx);
  _in.insert(_in.end(), data, data + n);
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

size_t TcpSerialInterface::writeFrame(const uint8_t src[], size_t len) {
  int fd = _client;
  if (fd < 0 || len == 0 || len > MAX_PAYLOAD) return 0;
  // One buffer, one write loop: a short write that split the 3-byte header
  // from its payload would desync the stream for every frame after it.
  uint8_t buf[3 + MAX_PAYLOAD];
  buf[0] = FRAME_TO_APP;
  buf[1] = (uint8_t)(len & 0xFF);
  buf[2] = (uint8_t)(len >> 8);
  memcpy(buf + 3, src, len);

  size_t total = 3 + len, off = 0;
  while (off < total) {
    ssize_t n = write(fd, buf + off, total - off);
    if (n > 0) { off += (size_t)n; continue; }
    if (n < 0 && errno == EINTR) continue;
    // Timed out or errored mid-frame: the stream is unrecoverable and the
    // client is not keeping up, so drop it rather than stall the mesh loop.
    fprintf(stderr, "[tcp] write failed (%s), dropping client\n", strerror(errno));
    dropClient(fd);
    return 0;
  }
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
