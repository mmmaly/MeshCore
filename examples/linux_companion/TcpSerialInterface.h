#pragma once
#include <helpers/BaseSerialInterface.h>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>
#include <atomic>

// BaseSerialInterface over TCP, using MeshCore's own serial framing:
// [start][len_lo][len_hi][payload], 0x3C app->node, 0x3E node->app.
// This is what lets the stock companion firmware talk to the phone app
// over a socket instead of BLE/USB.
class TcpSerialInterface : public BaseSerialInterface {
public:
  explicit TcpSerialInterface(int port) : _port(port) {}
  ~TcpSerialInterface() { stop(); }

  bool start();          // opens the listener + accept thread
  void stop();

  void enable() override { _enabled = true; }
  void disable() override { _enabled = false; }
  bool isEnabled() const override { return _enabled; }
  bool isConnected() const override { return _client >= 0; }
  bool isWriteBusy() const override { return false; }
  size_t writeFrame(const uint8_t src[], size_t len) override;
  size_t checkRecvFrame(uint8_t dest[]) override;
  size_t takePrivateFrame(uint8_t dest[], size_t max);   // frames with code >= 0xF0 (see RangingControl.h)

private:
  void acceptLoop();
  void consume(const uint8_t* data, size_t n);   // feed the frame parser
  void dropClient(int fd);                       // shutdown; accept thread closes
  int _port;
  int _srv = -1;
  std::atomic<int> _client{-1};
  std::atomic<bool> _enabled{false}, _running{false};
  std::thread _accept;
  std::mutex _mtx;
  std::vector<uint8_t> _in;                 // raw bytes from the socket
  std::deque<std::vector<uint8_t>> _frames; // decoded command frames
  std::deque<std::vector<uint8_t>> _private; // private (>= 0xF0) frames for main()
};
