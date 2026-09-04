import re
c="CMakeLists.txt"; t=open(c).read()
t=t.replace("LORA_FREQ=869.618f LORA_BW=62.5f LORA_SF=8 LORA_CR=5 LORA_TX_POWER=22","LORA_FREQ=869.432f LORA_BW=62.5f LORA_SF=7 LORA_CR=5 LORA_TX_POWER=22")
open(c,"w").write(t)
p="src/target.cpp"; s=open(p).read()
table='''
/* Sub-GHz PA settings Semtech measured on the Wio-LR2021 (usp_zephyr shield dtsi,
 * tx-power-cfg-lf), indexed by dBm+9 as RadioLib expects. RadioLib's built-in
 * table flattens above +14 dBm on this module and is 5 dB down at +22 (measured
 * with a second kit as monitor); Semtech's keeps 1 dB/dB to the top. */
static LR2021PaTableEntry_t wio_pa_lf[32] = {
  {2, 5, -13}, {6, 1, -13}, {6, 0, -6}, {1, 0, 4},   {2, 0, 4},   {1, 3, 2},   {0, 0, 14}, {0, 3, 9},
  {3, 0, 11},  {1, 0, 16},  {7, 0, 11}, {2, 0, 18},  {5, 0, 16},  {7, 0, 17},  {1, 2, 21}, {3, 0, 25},
  {0, 1, 32},  {2, 0, 32},  {3, 1, 27}, {2, 1, 32},  {5, 1, 28},  {5, 1, 30},  {4, 1, 34}, {5, 4, 31},
  {4, 4, 34},  {5, 6, 34},  {3, 5, 39}, {6, 6, 37},  {5, 5, 40},  {7, 4, 41},  {7, 4, 43}, {7, 7, 44},
};
'''
anchor="static CustomLR2021Wrapper s_radio(s_lora, board);"
assert anchor in s; s=s.replace(anchor, anchor+"\n"+table,1)
old="\ts_lora.setIrqDio(LR2021_IRQ_DIO);"
assert old in s; s=s.replace(old, old+"\n\ts_lora.setPaTable(wio_pa_lf, false);   /* before begin(): setOutputPower() reads it */",1)
open(p,"w").write(s); print("patched")
