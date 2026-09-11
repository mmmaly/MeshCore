#pragma once

#include "CustomLR2021.h"
#include "../MultiSfMemory.h"
#include "RadioLibWrappers.h"

// MeshCore wrapper for the LR2021. Implements the RadioLibWrapper hooks the mesh
// engine needs, using the LR2021's RadioLib API (getRssiInst / getRSSI / getSNR /
// boosted-gain). doResetAGC() is left to the base-class default for now; add an
// LR2021-specific reset only if RX sensitivity degrades over long runs.

class CustomLR2021Wrapper : public RadioLibWrapper {
public:
  CustomLR2021Wrapper(CustomLR2021& radio, mesh::MainBoard& board)
    : RadioLibWrapper(radio, board) { }

  // Multi-SF: a neighbour heard on a side detector is answered on its own SF
  // (see MultiSfMemory.h). One frame at a time: the override is programmed
  // right before startTransmit() and undone in onSendFinished().
  MultiSfMemory _sfmem;
  uint8_t _cur_sf = LORA_SF;
  uint8_t _tx_sf = 0;
  void setCurrentSf(uint8_t sf) { _cur_sf = sf; }
  // Airtime estimates (Dispatcher sets its TX timeout from this BEFORE the frame
  // reaches startSendRaw): assume the highest SF an override could pick.
  uint8_t getSpreadingFactor() const override { return _tx_sf ? _tx_sf : _sfmem.maxLiveSf(_cur_sf, millis()); }

  int recvRaw(uint8_t* bytes, int sz) override {
    int len = RadioLibWrapper::recvRaw(bytes, sz);
    if (len > 0) {
      uint8_t sf = ((CustomLR2021 *)_radio)->lastRxSf();
      if (sf) MESH_DEBUG_PRINTLN("LR2021: rx on side SF%u", sf);
      _sfmem.noteRx(bytes, len, sf ? sf : _cur_sf, _cur_sf, millis());
    }
    return len;
  }
  bool startSendRaw(const uint8_t* bytes, int len) override {
    CustomLR2021* r = (CustomLR2021 *)_radio;
    const char* why = "";
    uint8_t sf = _sfmem.pickTx(bytes, len, _cur_sf, millis(), &why);
    if (sf && sf != _cur_sf) {
      r->standby();
      if (r->setSpreadingFactor(sf) == RADIOLIB_ERR_NONE) {
        r->setPreambleLength(preambleLengthForSF(sf));
        _tx_sf = sf;
        MESH_DEBUG_PRINTLN("LR2021: tx at SF%u (%s)", sf, why);
      }
    }
    bool ok = RadioLibWrapper::startSendRaw(bytes, len);
    if (!ok && _tx_sf) restoreAfterOverride();
    return ok;
  }
  void onSendFinished() override {
    RadioLibWrapper::onSendFinished();
    if (_tx_sf) restoreAfterOverride();
    _radio->setPreambleLength(16);  // overcomes weird issues with small and big pkts
  }
  void restoreAfterOverride() {
    CustomLR2021* r = (CustomLR2021 *)_radio;
    r->standby();
    r->setSpreadingFactor(_cur_sf);
    r->setPreambleLength(preambleLengthForSF(_cur_sf));
    r->applySideDetectors();
    _tx_sf = 0;
  }

  void setParams(float freq, float bw, uint8_t sf, uint8_t cr) override {
    CustomLR2021* r = (CustomLR2021 *)_radio;
    r->setFrequency(freq);
    r->setSpreadingFactor(sf);
    r->setBandwidth(bw);
    r->setCodingRate(cr);
    r->applySideDetectors();
    _cur_sf = sf;
    // A band change (sub-GHz <-> 2.4 GHz) needs the RX front-end path and the PA
    // re-selected; RadioLib only does that through these two calls.
    r->setRxBoostedGainMode(r->getRxBoostLevel());
    r->setOutputPower(r->lastPowerDbm());
    updatePreamble(sf);
    // The setters above are SPI commands: in duty-cycled RX they wake the chip and end
    // the cycle, leaving it in standby while our state still says RX. Re-arm.
    RadioLibWrapper::idle();
  }

  bool isReceivingPacket() override {
    // In low-power mode the chip duty-cycles and an SPI poll would wake it and
    // end the cycle; report "busy" so the base class skips its noise-floor
    // sampling. The interrupt still delivers packets.
    if (((CustomLR2021 *)_radio)->lowPower && isInRecvMode()) return true;
    return ((CustomLR2021 *)_radio)->isReceiving();
  }

  // Dispatcher asks this right before a transmit. The poll may have woken the
  // chip out of its duty cycle; if nothing is in flight, re-arm so a deferred
  // transmit does not leave the receiver parked.
  bool isReceiving() override {
    CustomLR2021* r = (CustomLR2021 *)_radio;
    bool busy = r->isReceiving();
    if (r->lowPower && !busy && isInRecvMode()) { r->standby(); r->startReceive(); }
    return busy;
  }

  void onBeforeStartRecv() override {
    // re-arming (setRxPath) while still in continuous RX -> CMD_PERR (-706)
    // and a wedged receiver; drop to standby before every startReceive()
    _radio->standby();
  }

  float getCurrentRSSI() override {
    float rssi = -110;
    ((CustomLR2021 *)_radio)->getRssiInst(&rssi);
    return rssi;
  }

  float getLastRSSI() const override { return ((CustomLR2021 *)_radio)->getRSSI(); }
  float getLastSNR() const override { return ((CustomLR2021 *)_radio)->getSNR(); }

  void setRxBoostedGainMode(bool en) override {
    ((CustomLR2021 *)_radio)->setRxBoostedGainMode(en ? LR2021_RX_BOOST_LEVEL : 0);
  }
  bool getRxBoostedGainMode() const override {
    return ((CustomLR2021 *)_radio)->getRxBoostLevel() != 0;
  }
};
