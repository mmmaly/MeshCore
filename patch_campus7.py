# 1) CustomLR2021: duty-cycled receive in low-power mode
p="src/helpers/radiolib/CustomLR2021.h"; s=open(p).read()
old="""#if RADIOLIB_GODMODE
  int16_t startReceive() override {
    // re-assert max payload length before every RX: a TX leaves the chip's
    // packet-length param at the last TX size, which would clip longer
    // incoming packets. Needs GODMODE (setLoRaPacketParams is private).
    setLoRaPacketParams(this->preambleLengthLoRa, this->headerType,
                        RADIOLIB_LR2021_MAX_PACKET_LENGTH, this->crcTypeLoRa,
                        this->invertIQEnabled);
    return LR2021::startReceive();
  }
#endif"""
new="""  // Low-power (battery) mode: receive with the chip's RX duty cycling - it sleeps
  // between short listening windows and only stays awake once a preamble is
  // detected. Sized for the mesh's preambles (32 symbols at SF<=8). Any SPI
  // command during the sleep phase wakes the chip and ends the cycle, so the
  // wrapper must not poll while in this mode (see CustomLR2021Wrapper).
  bool lowPower = false;
  uint16_t lpSenderPreamble = 32;
  uint8_t  lpMinSymbols = 6;

#if RADIOLIB_GODMODE
  int16_t startReceive() override {
    // re-assert max payload length before every RX: a TX leaves the chip's
    // packet-length param at the last TX size, which would clip longer
    // incoming packets. Needs GODMODE (setLoRaPacketParams is private).
    setLoRaPacketParams(this->preambleLengthLoRa, this->headerType,
                        RADIOLIB_LR2021_MAX_PACKET_LENGTH, this->crcTypeLoRa,
                        this->invertIQEnabled);
    if (lowPower) {
      uint16_t pre = lpSenderPreamble > this->preambleLengthLoRa ? this->preambleLengthLoRa : lpSenderPreamble;
      return LR2021::startReceiveDutyCycleAuto(pre, lpMinSymbols);
    }
    return LR2021::startReceive();
  }
#endif"""
assert old in s; s=s.replace(old,new,1); open(p,"w").write(s)

# 2) wrapper: no SPI polling while duty cycling; re-arm after the one poll TX needs
w="src/helpers/radiolib/CustomLR2021Wrapper.h"; t=open(w).read()
old2="""  bool isReceivingPacket() override {
    return ((CustomLR2021 *)_radio)->isReceiving();
  }"""
new2="""  bool isReceivingPacket() override {
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
  }"""
assert old2 in t; t=t.replace(old2,new2,1); open(w,"w").write(t)

# 3) main.cpp: event-driven loop
m="zephyr-port/07_companion/src/main.cpp"; s=open(m).read()
old3="#include \"MyMesh.h\"   /* examples/companion_radio (on the include path) */"
new3="""#include "MyMesh.h"   /* examples/companion_radio (on the include path) */

/* Wake-ups for the main loop: the LoRa DIO interrupt and BLE data give this
 * semaphore; otherwise the loop sleeps (MC_LOOP_SLEEP_MS) and the SoC idles. */
K_SEM_DEFINE(mc_wake_sem, 0, 1);
extern "C" void mc_wake(void) { k_sem_give(&mc_wake_sem); }
#ifndef MC_LOOP_SLEEP_MS
#ifdef MC_LOW_POWER
#define MC_LOOP_SLEEP_MS 25
#else
#define MC_LOOP_SLEEP_MS 2
#endif
#endif"""
assert old3 in s; s=s.replace(old3,new3,1)
old4="\t\tk_msleep(1);   /* snappy serial/radio polling for the app's frame bursts */"
new4="\t\tk_sem_take(&mc_wake_sem, K_MSEC(MC_LOOP_SLEEP_MS));   /* radio IRQ / BLE data wake us early */"
assert old4 in s; s=s.replace(old4,new4,1)
old5="\tif (!radio_init()) { printk(\"radio_init FAILED\\n\"); return 0; }"
new5="""\tif (!radio_init()) { printk("radio_init FAILED\\n"); return 0; }
#ifdef MC_LOW_POWER
\tradio_set_low_power(true);
\tprintk("low-power build: LR2021 RX duty cycling, %d ms loop sleep\\n", MC_LOOP_SLEEP_MS);
#endif"""
assert old5 in s; s=s.replace(old5,new5,1)
open(m,"w").write(s)

# 4) target: low-power switch
t2="zephyr-port/07_companion/src/target.cpp"; s=open(t2).read()
s += '''
void radio_set_low_power(bool on)
{
	s_lora.lowPower = on;
	s_radio.begin();   /* wrapper re-arms RX with the new mode */
}
'''
open(t2,"w").write(s)
h="zephyr-port/07_companion/src/target.h"; s=open(h).read()
s=s.replace("mesh::LocalIdentity radio_new_identity();","mesh::LocalIdentity radio_new_identity();\nvoid radio_set_low_power(bool on);",1)
open(h,"w").write(s)

# 5) HAL: wake the loop from the DIO interrupt
hal="zephyr-port/07_companion/src/zephyr_radiolib_hal.h"; s=open(hal).read()
old6="    static void gpioIsr(const struct device *, struct gpio_callback *, uint32_t) {\n        if (user_cb) user_cb();\n    }"
new6="    static void gpioIsr(const struct device *, struct gpio_callback *, uint32_t) {\n        if (user_cb) user_cb();\n        mc_wake();\n    }"
assert old6 in s; s=s.replace(old6,new6,1)
s=s.replace("#include <RadioLib.h>","#include <RadioLib.h>\nextern \"C\" void mc_wake(void);   /* main loop wake-up (main.cpp) */",1)
open(hal,"w").write(s)

# 6) BLE rx wakes the loop
b="zephyr-port/07_companion/src/serial_ble_interface.cpp"; s=open(b).read()
import re
i=s.index("void SerialBLEInterface::_onRx(")
j=s.index("{", i)
s=s[:j+1]+"\n\textern void mc_wake(void);\n\tmc_wake();"+s[j+1:]
open(b,"w").write(s)

# 7) CMake option + prj.conf PM
c="zephyr-port/07_companion/CMakeLists.txt"; s=open(c).read()
s=s.replace('option(MC_BAND_2G4 "Boot on the 2.4 GHz LoRa preset instead of sub-GHz" OFF)','''option(MC_BAND_2G4 "Boot on the 2.4 GHz LoRa preset instead of sub-GHz" OFF)
option(MC_LOW_POWER "Battery build: LR2021 RX duty cycling + sleepy main loop" OFF)
if(MC_LOW_POWER)
  target_compile_definitions(app PRIVATE MC_LOW_POWER=1)
endif()''')
open(c,"w").write(s)
pc="zephyr-port/07_companion/prj.conf"; s=open(pc).read()
s += "\n# Let the SoC idle between events (the main loop now blocks on a semaphore)\nCONFIG_PM=y\n"
open(pc,"w").write(s)
print("patched7")
