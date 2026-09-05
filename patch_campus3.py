p="src/helpers/radiolib/CustomLR2021.h"; s=open(p).read()
old="""  int16_t setRxBoostedGainMode(uint8_t level) {"""
new="""#if RADIOLIB_GODMODE
  // 2.4 GHz PA: RadioLib's setOutputPower() sends the LF duty field as 6 for the
  // HF PA (the chip wants 0) and selects the PA before programming a power in the
  // HF range - fw 1.24 answers CMD_PERR to both. Program Semtech's measured
  // Wio-LR2021 HF table directly, TX power first, then the PA selection.
  struct HfPaEntry { int8_t half_power; uint8_t duty; };
  int16_t setOutputPower(int8_t power) override {
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

  int16_t setRxBoostedGainMode(uint8_t level) {"""
assert old in s; s=s.replace(old,new,1); open(p,"w").write(s)

w="src/helpers/radiolib/CustomLR2021Wrapper.h"; t=open(w).read()
old2="""  void setParams(float freq, float bw, uint8_t sf, uint8_t cr) override {
    ((CustomLR2021 *)_radio)->setFrequency(freq);
    ((CustomLR2021 *)_radio)->setSpreadingFactor(sf);
    ((CustomLR2021 *)_radio)->setBandwidth(bw);
    ((CustomLR2021 *)_radio)->setCodingRate(cr);
    updatePreamble(sf);
  }"""
new2="""  void setParams(float freq, float bw, uint8_t sf, uint8_t cr) override {
    CustomLR2021* r = (CustomLR2021 *)_radio;
    r->setFrequency(freq);
    r->setSpreadingFactor(sf);
    r->setBandwidth(bw);
    r->setCodingRate(cr);
    // A band change (sub-GHz <-> 2.4 GHz) needs the RX front-end path and the PA
    // re-selected; RadioLib only does that through these two calls.
    r->setRxBoostedGainMode(r->getRxBoostLevel());
    r->setOutputPower(_tx_dbm);
    updatePreamble(sf);
  }
  void setTxPower(uint8_t dbm) override { _tx_dbm = (int8_t)dbm; RadioLibWrapper::setTxPower(dbm); }"""
assert old2 in t; t=t.replace(old2,new2,1)
old3="class CustomLR2021Wrapper : public RadioLibWrapper {\npublic:"
new3="class CustomLR2021Wrapper : public RadioLibWrapper {\n  int8_t _tx_dbm = LORA_TX_POWER;\npublic:"
assert old3 in t; t=t.replace(old3,new3,1); open(w,"w").write(t); print("patched3")
