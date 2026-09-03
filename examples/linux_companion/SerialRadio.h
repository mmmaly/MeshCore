#pragma once
#include "HostRadio.h"
#include <atomic>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// A mesh::Radio whose PHY is a real LoRa transceiver on a serial port: the
// lr2021_serial firmware (this directory, lr2021_serial/) on the Semtech
// LR2021 EVK (XIAO nRF54L15 + Wio-LR2021). Same slot as SdrRadio, same
// "rx cfg:" / "rx ok:" line contract for reception; transmit is a "tx <hex>"
// line answered by "tx done" / "tx err".
//
// What the LR2021 adds over the SDR pair: hardware side detectors, i.e. up to
// three extra spreading factors received in parallel on the same channel
// (side_sfs), and a +22 dBm PA. What it loses: several frequencies at once.
class SerialRadio : public HostRadio {
public:
  struct Config {
    std::string device = "/dev/ttyACM0";
    int baud = 115200;
    uint32_t freq = 869432000, bw = 62500;
    int sf = 7, cr = 5;              // cr in RadioLib terms: 5..8 = 4/5..4/8
    std::string side_sfs = "8";      // extra SFs to detect in parallel ("" = none)
    int tx_power = 22;               // dBm, -9..22 on the sub-GHz PA
    int rx_boost = 7;                // 0 = off, 7 = max (Semtech default)
  };

  explicit SerialRadio(const Config& cfg) : _cfg(cfg) {}
  SerialRadio() = default;
  ~SerialRadio() { stop(); }

  // Only safe before begin() starts the reader thread (see SdrRadio).
  Config& config() { return _cfg; }

  void begin() override;
  void stop();

  int recvRaw(uint8_t* bytes, int sz) override;
  bool startSendRaw(const uint8_t* bytes, int len) override;
  bool isSendComplete() override { return true; }   // startSendRaw waits for "tx done"
  void onSendFinished() override {}
  bool isInRecvMode() const override { return true; }
  uint32_t getEstAirtimeFor(int len_bytes) override;
  float packetScore(float snr, int packet_len) override;
  float getLastSNR() const override { return _last_snr; }
  float getLastRSSI() const override { return _last_rssi; }

  void setParams(float freq_mhz, float bw_khz, uint8_t sf, uint8_t cr) override;
  void setTxPower(uint8_t dbm) override;
  void setRxBoostedGainMode(bool state) override;
  bool getRxBoostedGainMode() const override;

  uint32_t getPacketsRecv() const override { return _n_recv; }
  uint32_t getPacketsSent() const override { return _n_sent; }
  uint32_t getPacketsRecvErrors() const override { return _n_err; }

private:
  void readerLoop();
  bool openPort();
  void handleLine(char* line);
  bool sendLine(const std::string& line);
  std::string configLine() const;     // caller holds _cfg_mtx
  void sendConfig();

  struct RxPacket {
    std::vector<uint8_t> bytes;
    float snr, rssi;
  };

  Config _cfg;
  mutable std::mutex _cfg_mtx;
  std::thread _reader;
  std::atomic<bool> _running{false};
  std::atomic<int> _fd{-1};
  std::mutex _wr_mtx;                 // one writer at a time on the port

  std::mutex _mtx;
  std::deque<RxPacket> _rx;
  std::atomic<float> _last_snr{0.0f}, _last_rssi{-120.0f};
  std::atomic<uint32_t> _n_recv{0}, _n_sent{0}, _n_err{0};

  std::mutex _tx_mtx;
  std::condition_variable _tx_cv;
  int _tx_result = 0;                 // 0 pending, 1 done, -1 error
};
