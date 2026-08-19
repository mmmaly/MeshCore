#pragma once
#include <Dispatcher.h>       // mesh::Radio
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <atomic>

// A mesh::Radio whose PHY is a pair of SDRs: an RTL-SDR receiving through
// lora_rx, a HackRF transmitting through lora_tx. This is the whole point of
// the port - it slots in exactly where a CustomSX1262Wrapper would, so all
// of MyMesh.cpp / BaseChatMesh.cpp / Mesh.cpp run unmodified above it.
class SdrRadio : public mesh::Radio {
public:
  struct Config {
    std::string rx_binary = "lora_rx", tx_binary = "lora_tx";
    std::string rx_device, rx_channels = "869618000", rx_sfs = "8";
    uint32_t bw = 62500, tx_freq = 869618000;
    int rx_ppm = 0, tx_sf = 8, tx_cr = 1, tx_ppm = 0, tx_vga = 47;
    bool rx_agc = true, tx_amp = true;
    double tx_duty = 10.0;
  };

  explicit SdrRadio(const Config& cfg) : _cfg(cfg) {}
  SdrRadio() = default;

  // Mutable access: this object is a global constructed before main() runs,
  // so command-line settings have to be applied to it, not to whatever
  // config it was constructed from.
  Config& config() { return _cfg; }
  ~SdrRadio() { stop(); }

  void begin() override;
  void stop();

  int recvRaw(uint8_t* bytes, int sz) override;
  bool startSendRaw(const uint8_t* bytes, int len) override;
  bool isSendComplete() override { return true; }   // lora_tx is synchronous
  void onSendFinished() override {}
  bool isInRecvMode() const override { return true; }
  uint32_t getEstAirtimeFor(int len_bytes) override;
  float packetScore(float snr, int packet_len) override;
  float getLastSNR() const override { return _last_snr; }

  // Called by MyMesh when the app changes radio settings. Retuning the
  // transmitter is immediate; the receiver is restarted so lora_rx picks up
  // the new frequency/SF (an SDR can watch several at once, so the new
  // channel is added rather than replacing the list).
  void setParams(float freq_mhz, float bw_khz, uint8_t sf, uint8_t cr);
  void setTxPower(uint8_t dbm);
  void setRxBoostedGainMode(bool state) { _cfg.rx_agc = state; }
  bool getRxBoostedGainMode() const { return _cfg.rx_agc; }

  // Counters the companion's radio stats report
  uint32_t getPacketsRecv() const { return _n_recv; }
  uint32_t getPacketsSent() const { return _n_sent; }
  uint32_t getPacketsRecvErrors() const { return 0; }  // lora_rx only emits CRC-valid frames
  float getLastRSSI() const override { return -105.0f + _last_snr; }

private:
  void superviseLoop();
  void readPipe(int fd);
  std::vector<std::string> rxArgv() const;
  Config _cfg;
  std::thread _reader;
  std::atomic<bool> _running{false};
  std::mutex _mtx;
  std::deque<std::vector<uint8_t>> _rx;   // complete packets from lora_rx
  float _last_snr = 0.0f;
  uint32_t _n_recv = 0, _n_sent = 0;
  pid_t _rx_pid = -1;
  int _rx_fd = -1;
};
