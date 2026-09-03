#pragma once
#include <Dispatcher.h>       // mesh::Radio

// What MyMesh calls on `radio_driver` beyond the mesh::Radio contract - the
// part of RadioLibWrapper's surface the companion firmware actually uses.
// Both host radios (SdrRadio: RTL-SDR + HackRF; SerialRadio: LR2021 EVK over
// USB serial) implement it.
class HostRadio : public mesh::Radio {
public:
  virtual void setParams(float freq_mhz, float bw_khz, uint8_t sf, uint8_t cr) = 0;
  virtual void setTxPower(uint8_t dbm) = 0;
  virtual void setRxBoostedGainMode(bool state) = 0;
  virtual bool getRxBoostedGainMode() const = 0;
  virtual uint32_t getPacketsRecv() const = 0;
  virtual uint32_t getPacketsSent() const = 0;
  virtual uint32_t getPacketsRecvErrors() const = 0;
};

// The companion's globals (the_mesh, and radio_driver they bind to) are
// constructed before main() sees argv, so the backend is chosen at run time
// behind this forwarder. select() must precede begin().
class RadioProxy : public HostRadio {
public:
  void select(HostRadio* r) { _r = r; }
  HostRadio* backend() { return _r; }

  void begin() override { _r->begin(); }
  int recvRaw(uint8_t* bytes, int sz) override { return _r->recvRaw(bytes, sz); }
  uint32_t getEstAirtimeFor(int len_bytes) override { return _r->getEstAirtimeFor(len_bytes); }
  float packetScore(float snr, int packet_len) override { return _r->packetScore(snr, packet_len); }
  bool startSendRaw(const uint8_t* bytes, int len) override { return _r->startSendRaw(bytes, len); }
  bool isSendComplete() override { return _r->isSendComplete(); }
  void onSendFinished() override { _r->onSendFinished(); }
  void loop() override { _r->loop(); }
  bool isInRecvMode() const override { return _r->isInRecvMode(); }
  bool isReceiving() override { return _r->isReceiving(); }
  float getLastRSSI() const override { return _r->getLastRSSI(); }
  float getLastSNR() const override { return _r->getLastSNR(); }

  void setParams(float freq_mhz, float bw_khz, uint8_t sf, uint8_t cr) override { _r->setParams(freq_mhz, bw_khz, sf, cr); }
  void setTxPower(uint8_t dbm) override { _r->setTxPower(dbm); }
  void setRxBoostedGainMode(bool state) override { _r->setRxBoostedGainMode(state); }
  bool getRxBoostedGainMode() const override { return _r->getRxBoostedGainMode(); }
  uint32_t getPacketsRecv() const override { return _r->getPacketsRecv(); }
  uint32_t getPacketsSent() const override { return _r->getPacketsSent(); }
  uint32_t getPacketsRecvErrors() const override { return _r->getPacketsRecvErrors(); }

private:
  HostRadio* _r = nullptr;
};
