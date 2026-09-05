p="src/helpers/radiolib/CustomLR2021.h"; s=open(p).read()
s=s.replace("""  int16_t setOutputPower(int8_t power) override {
    if (!this->highFreq) {""","""  int8_t _last_dbm = LORA_TX_POWER;
  int8_t lastPowerDbm() const { return _last_dbm; }
  int16_t setOutputPower(int8_t power) override {
    _last_dbm = power;
    if (!this->highFreq) {""",1)
open(p,"w").write(s)
w="src/helpers/radiolib/CustomLR2021Wrapper.h"; t=open(w).read()
t=t.replace("    r->setOutputPower(_tx_dbm);","    r->setOutputPower(r->lastPowerDbm());")
t=t.replace("\n  void setTxPower(uint8_t dbm) override { _tx_dbm = (int8_t)dbm; RadioLibWrapper::setTxPower(dbm); }","")
t=t.replace("class CustomLR2021Wrapper : public RadioLibWrapper {\n  int8_t _tx_dbm = LORA_TX_POWER;\npublic:","class CustomLR2021Wrapper : public RadioLibWrapper {\npublic:")
open(w,"w").write(t); print("patched4")
