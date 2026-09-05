/*
 * LR2021 RTToF (time-of-flight ranging) on top of RadioLib, for the Zephyr
 * companion. Same sequence as lr2021_serial's rng commands / Semtech's USP demo:
 * packet type RTToF, the LoRa modulation re-programmed, the PLL frequency-step
 * register workaround, Tx->Rx delay indicator, then SetTx (manager) or continuous
 * SetRx (subordinate). Distance [m] = raw * 150 / (4096 * BW_MHz).
 */
#pragma once
#include <zephyr/kernel.h>
#include <helpers/radiolib/CustomLR2021.h>
#include "RangingControl.h"

static int16_t rttof_setup(CustomLR2021& r, bool subordinate, uint32_t addr, uint32_t delay, int8_t pwr, uint8_t irq_dio) {
  r.standby();
  int16_t st = r.setPacketType(RADIOLIB_LR2021_PACKET_TYPE_RTTOF);
  if (st == RADIOLIB_ERR_NONE) st = r.setLoRaModulationParams(r.sfCode(), r.bwCode(), r.crCode(), r.ldroCode());
  if (st == RADIOLIB_ERR_NONE) {          // workaround: truncate the PLL frequency step
    uint32_t v = 0; st = r.readRegMem32(0x00F40144, &v, 1);
    if (st == RADIOLIB_ERR_NONE) { v &= ~0x7Fu; st = r.writeRegMem32(0x00F40144, &v, 1); }
  }
  if (st == RADIOLIB_ERR_NONE && !subordinate) st = r.setOutputPower(pwr);
  if (st == RADIOLIB_ERR_NONE) st = r.setLoRaPacketParams(12, RADIOLIB_LRXXXX_LORA_HEADER_EXPLICIT, 7, RADIOLIB_LRXXXX_LORA_CRC_ENABLED, 0);
  if (st == RADIOLIB_ERR_NONE) st = r.setLoRaSyncword(RADIOLIB_LR2021_LORA_SYNC_WORD_PRIVATE);
  uint32_t irq = subordinate ? (RADIOLIB_LR2021_IRQ_TIMEOUT | RADIOLIB_LR2021_IRQ_RNG_REQ_DIS | RADIOLIB_LR2021_IRQ_RNG_RESP_DONE | RADIOLIB_LR2021_IRQ_RNG_REQ_VALID)
                             : (RADIOLIB_LR2021_IRQ_RNG_EXCH_VALID | RADIOLIB_LR2021_IRQ_RNG_TIMEOUT | RADIOLIB_LR2021_IRQ_TIMEOUT);
  if (st == RADIOLIB_ERR_NONE) st = r.setDioIrqConfig(irq_dio, irq);
  if (st == RADIOLIB_ERR_NONE) st = r.setRangingTxRxDelay(delay);
  if (st == RADIOLIB_ERR_NONE) st = r.setRangingParams(false, 15);
  if (st == RADIOLIB_ERR_NONE) st = subordinate ? r.setRangingAddr(addr, 4) : r.setRangingReqAddr(addr);
  if (st == RADIOLIB_ERR_NONE) st = r.clearIrqState(RADIOLIB_LR2021_IRQ_ALL);
  return st;
}

// Apply the ranging radio parameters in LoRa mode first (RadioLib caches the codes).
static int16_t rttof_tune(CustomLR2021& r, const RangingRequest& req, int8_t pwr) {
  r.standby();
  int16_t st = r.setFrequency(req.freq_hz / 1000000.0f);
  if (st == RADIOLIB_ERR_NONE) st = r.setBandwidth(req.bw_hz / 1000.0f);
  if (st == RADIOLIB_ERR_NONE) st = r.setSpreadingFactor(req.sf);
  if (st == RADIOLIB_ERR_NONE) st = r.setCodingRate(5);
  if (st == RADIOLIB_ERR_NONE) st = r.setRxBoostedGainMode(7);   // also selects the band's RX path
  if (st == RADIOLIB_ERR_NONE) st = r.setOutputPower(pwr);        // and the band's PA
  return st;
}

// Answer ranging requests for `window_ms`. Blocks. Leaves the chip in standby, packet type LoRa.
static bool rttof_subordinate(CustomLR2021& r, const RangingRequest& req, uint32_t my_addr, uint8_t irq_dio, uint32_t window_ms, int* answered) {
  int8_t pwr = req.freq_hz > 1500000000u ? 10 : 14;
  if (rttof_tune(r, req, pwr) != RADIOLIB_ERR_NONE) return false;
  if (rttof_setup(r, true, my_addr, req.delay, pwr, irq_dio) != RADIOLIB_ERR_NONE) return false;
  int n = 0;
  int64_t end = k_uptime_get() + window_ms;
  r.setRx(RADIOLIB_LR2021_RX_TIMEOUT_INF);
  while (k_uptime_get() < end) {
    uint32_t irq = r.getIrqFlags();
    if (irq & (RADIOLIB_LR2021_IRQ_RNG_RESP_DONE | RADIOLIB_LR2021_IRQ_RNG_REQ_DIS | RADIOLIB_LR2021_IRQ_TIMEOUT)) {
      if (irq & RADIOLIB_LR2021_IRQ_RNG_RESP_DONE) n++;
      r.clearIrqState(RADIOLIB_LR2021_IRQ_ALL);
      r.setRx(RADIOLIB_LR2021_RX_TIMEOUT_INF);
    }
    k_msleep(2);
  }
  r.standby();
  r.clearTxFifo(); r.clearRxFifo();
  r.setPacketType(RADIOLIB_LR2021_PACKET_TYPE_LORA);
  if (answered) *answered = n;
  return true;
}

// Run `count` exchanges as manager against `peer_addr`. Blocks. Leaves the chip in standby, packet type LoRa.
static bool rttof_manager(CustomLR2021& r, const RangingRequest& req, uint32_t peer_addr, uint8_t irq_dio, RangingResult& res) {
  res = RangingResult(); res.count = req.count;
  int8_t pwr = req.freq_hz > 1500000000u ? 10 : 14;
  if (rttof_tune(r, req, pwr) != RADIOLIB_ERR_NONE || rttof_setup(r, false, peer_addr, req.delay, pwr, irq_dio) != RADIOLIB_ERR_NONE) { res.status = 2; return false; }
  static const uint8_t payload[7] = {0x52, 0x4e, 0x47, 0, 0, 0, 0};
  static int32_t raws[200]; int n = 0;
  float bw_mhz = req.bw_hz / 1000000.0f;
  for (int i = 0; i < req.count && i < 200; i++) {
    r.standby(); r.clearIrqState(RADIOLIB_LR2021_IRQ_ALL);
    r.clearTxFifo();                                  // stale FIFO bytes corrupt the next LoRa TX
    r.writeRadioTxFifo(payload, sizeof(payload));
    if (r.setTx(0) != RADIOLIB_ERR_NONE) break;
    int64_t t0 = k_uptime_get(); uint32_t irq = 0;
    while (k_uptime_get() - t0 < 1500) {
      irq = r.getIrqFlags();
      if (irq & (RADIOLIB_LR2021_IRQ_RNG_EXCH_VALID | RADIOLIB_LR2021_IRQ_RNG_TIMEOUT | RADIOLIB_LR2021_IRQ_TIMEOUT)) break;
      k_msleep(1);
    }
    if (irq & RADIOLIB_LR2021_IRQ_RNG_EXCH_VALID) {
      uint32_t r1 = 0, r2 = 0; uint8_t rs = 0;
      r.getRangingResult(RADIOLIB_LR2021_RANGING_RESULT_TYPE_RAW, &r1, &rs, &r2);
      int32_t v = (int32_t)r1; if (v >= (1 << 23)) v -= (1 << 24);
      raws[n++] = v;
    }
    r.clearIrqState(RADIOLIB_LR2021_IRQ_ALL);
    k_msleep(50);
  }
  r.standby();
  r.clearTxFifo(); r.clearRxFifo();
  r.setPacketType(RADIOLIB_LR2021_PACKET_TYPE_LORA);
  if (!n) { res.status = 1; return false; }
  for (int i = 1; i < n; i++) { int32_t v = raws[i]; int j = i - 1; while (j >= 0 && raws[j] > v) { raws[j + 1] = raws[j]; j--; } raws[j + 1] = v; }
  auto cm = [&](int32_t raw) { return (int32_t)(raw * 150.0f / (4096.0f * bw_mhz) * 100.0f); };
  res.status = 0; res.valid = (uint8_t)n;
  res.median_cm = cm(raws[n / 2]); res.min_cm = cm(raws[0]); res.max_cm = cm(raws[n - 1]);
  return true;
}
