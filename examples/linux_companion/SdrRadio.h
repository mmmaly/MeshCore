#pragma once
#include "HostRadio.h"
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
class SdrRadio : public HostRadio {
public:
  struct Config {
    std::string rx_binary = "lora_rx", tx_binary = "lora_tx";
    std::string rx_device, rx_channels = "869618000", rx_sfs = "8";
    uint32_t bw = 62500, tx_freq = 869618000;
    int rx_ppm = 0, tx_sf = 8, tx_cr = 1, tx_ppm = 0, tx_vga = 47;
    bool rx_agc = true, tx_amp = true;
    double tx_duty = 10.0;
    // A LoRa chirp is constant-envelope, so full DAC scale cannot clip and
    // costs nothing in signal quality: lora_tx defaults to 0.7 for callers
    // who may not know that, but a node wants every dB. Worth +3.1 dB.
    double tx_level = 1.0;
  };

  explicit SdrRadio(const Config& cfg) : _cfg(cfg) {}
  SdrRadio() = default;

  // Mutable access: this object is a global constructed before main() runs,
  // so command-line settings have to be applied to it, not to whatever
  // config it was constructed from. Only safe before begin() starts the
  // supervisor thread; afterwards _cfg belongs to _cfg_mtx.
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
  void setParams(float freq_mhz, float bw_khz, uint8_t sf, uint8_t cr) override;
  void setTxPower(uint8_t dbm) override;
  void setRxBoostedGainMode(bool state) override;
  bool getRxBoostedGainMode() const override;

  // Counters the companion's radio stats report
  uint32_t getPacketsRecv() const override { return _n_recv; }
  uint32_t getPacketsSent() const override { return _n_sent; }
  uint32_t getPacketsRecvErrors() const override { return 0; }  // lora_rx only emits CRC-valid frames
  float getLastRSSI() const override { return -105.0f + _last_snr; }

private:
  void superviseLoop();
  void readPipe(int fd);
  std::vector<std::string> rxArgv() const;

  // SNR travels with its packet. The reader thread parses ahead of the mesh
  // thread's recvRaw(), so a single "most recent SNR" field would hand the
  // popped packet a *later* packet's measurement - which then lands in the
  // app's signal report and in packetScore().
  struct RxPacket {
    std::vector<uint8_t> bytes;
    float snr;
  };

  Config _cfg;
  mutable std::mutex _cfg_mtx;            // _cfg: mesh thread writes, supervisor reads
  std::thread _reader;
  std::atomic<bool> _running{false};
  std::mutex _mtx;
  std::deque<RxPacket> _rx;               // complete packets from lora_rx
  std::atomic<float> _last_snr{0.0f};     // SNR of the packet recvRaw() last returned
  std::atomic<uint32_t> _n_recv{0}, _n_sent{0};
  std::atomic<pid_t> _rx_pid{-1};
  std::atomic<int> _rx_fd{-1};
};
