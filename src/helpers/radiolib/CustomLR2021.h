#pragma once

#include <RadioLib.h>
#include "MeshCore.h"

// Custom RadioLib driver for the Semtech LR2021 (LoRa Plus), as used on the
// Seeed Wio-LR2021 (LR2021 + nRF54L15). Two board/chip quirks are handled here
// (both discovered during hardware bring-up):


#ifndef LR2021_IRQ_DIO
  #define LR2021_IRQ_DIO 8        // Wio-LR2021 wires the LR2021's DIO8 to the host IRQ line
#endif

#ifndef LR2021_RX_BOOST_LEVEL
  #define LR2021_RX_BOOST_LEVEL 7 // matches Semtech usp_zephyr rx-boost-cfg = <7>
#endif

class CustomLR2021 : public LR2021 {
  uint8_t _rx_boost_level = 0;

public:
  CustomLR2021(Module *mod) : LR2021(mod) { }

  // route the host IRQ to the DIO the board actually wires (std_init does this
  // too; callers that do their own begin() sequence use this directly)
  void setIrqDio(uint8_t n) { irqDioNum = n; }

#if defined(ARDUINO)   // Arduino-core bring-up; Zephyr (compat shim, no SPIClass) drives begin() itself
  bool std_init(SPIClass* spi = NULL) {
    // route the host IRQ to the DIO the board actually wires (default DIO8)
    irqDioNum = LR2021_IRQ_DIO;

  #ifdef LORA_CR
    uint8_t cr = LORA_CR;
  #else
    uint8_t cr = 5;
  #endif

  #if defined(P_LORA_SCLK)
    #if defined(ESP32_PLATFORM)
      if (spi) spi->begin(P_LORA_SCLK, P_LORA_MISO, P_LORA_MOSI);
    #elif defined(NRF52_PLATFORM)
      if (spi) { spi->setPins(P_LORA_MISO, P_LORA_SCLK, P_LORA_MOSI); spi->begin(); }
    #else
      if (spi) spi->begin();   // bare-metal nRF54L15 core: SPI pins are fixed (D8/D9/D10)
    #endif
  #else
    if (spi) spi->begin();
  #endif

    // tcxoVoltage = 0 -> skip SetTcxoMode (RadioLib mis-scales its start_time; see note above).
    int status = begin(LORA_FREQ, LORA_BW, LORA_SF, cr,
                       RADIOLIB_LR2021_LORA_SYNC_WORD_PRIVATE, LORA_TX_POWER, 16, 0.0f);
    if (status != RADIOLIB_ERR_NONE) {
      Serial.print("ERROR: LR2021 init failed: ");
      Serial.println(status);
      return false;
    }

    setCRC(2);
    explicitHeader();

  #ifdef RX_BOOSTED_GAIN
    if (RX_BOOSTED_GAIN) setRxBoostedGainMode(LR2021_RX_BOOST_LEVEL);
  #endif

    return true;  // success
  }
#endif  // ARDUINO

  size_t getPacketLength(bool update) override {
    size_t len = LR2021::getPacketLength(update);
    if (len == 0 && (getIrqFlags() & RADIOLIB_LR2021_IRQ_LORA_HDR_CRC_ERROR)) {
      // corrupted header: return to a known-good state; recvRaw restarts RX
      MESH_DEBUG_PRINTLN("LR2021: got header CRC err, calling standby()");
      standby();
    }
    return len;
  }

#if RADIOLIB_GODMODE
  int16_t startReceive() override {
    // re-assert max payload length before every RX: a TX leaves the chip's
    // packet-length param at the last TX size, which would clip longer
    // incoming packets. Needs GODMODE (setLoRaPacketParams is private).
    setLoRaPacketParams(this->preambleLengthLoRa, this->headerType,
                        RADIOLIB_LR2021_MAX_PACKET_LENGTH, this->crcTypeLoRa,
                        this->invertIQEnabled);
    return LR2021::startReceive();
  }
#endif

  bool isReceiving() {
    uint32_t irq = getIrqFlags();
    return (irq & RADIOLIB_LR2021_IRQ_PREAMBLE_DETECTED)
        || (irq & RADIOLIB_LR2021_IRQ_LORA_HEADER_VALID);
  }

#if RADIOLIB_GODMODE
  // 2.4 GHz PA: RadioLib's setOutputPower() sends the LF duty field as 6 for the
  // HF PA (the chip wants 0) and selects the PA before programming a power in the
  // HF range - fw 1.24 answers CMD_PERR to both. Program Semtech's measured
  // Wio-LR2021 HF table directly, TX power first, then the PA selection.
  struct HfPaEntry { int8_t half_power; uint8_t duty; };
  int8_t _last_dbm = LORA_TX_POWER;
  int8_t lastPowerDbm() const { return _last_dbm; }
  int16_t setOutputPower(int8_t power) override {
    _last_dbm = power;
    if (!this->highFreq) {
      if (power > 22) power = 22; if (power < -9) power = -9;
      return LR2021::setOutputPower(power);
    }
    static const HfPaEntry hf[31] = {
      {-39,29}, {-39,29}, {-39,16}, {-35,19}, {-32,19}, {-29,19}, {-27,16}, {-24,17},
      {-22,16}, {-19,18}, {-17,16}, {-14,21}, {-12,18}, { -7,30}, { -8,16}, { -5,24},
      { -2,27}, {  1,29}, {  4,30}, {  6,30}, {  7,28}, {  8,25}, { 10,25}, { 15,31},
      { 16,30}, { 18,30}, { 21,31}, { 22,30}, { 24,30}, { 24,26}, { 24,16},
    };
    if (power > 12) power = 12; if (power < -18) power = -18;
    const HfPaEntry& e = hf[power + 18];
    int16_t st = setTxParams(e.half_power, RADIOLIB_LRXXXX_PA_RAMP_48U);
    if (st != RADIOLIB_ERR_NONE) return st;
    return setPaConfig(1, RADIOLIB_LR2021_PA_LF_MODE_FSM, 0, 7, e.duty);
  }
#endif

  int16_t setRxBoostedGainMode(uint8_t level) {
    _rx_boost_level = level;
    return LR2021::setRxBoostedGainMode(level);
  }

  uint8_t getRxBoostLevel() const { return _rx_boost_level; }
};
