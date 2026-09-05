t="zephyr-port/07_companion/src/target.cpp"; s=open(t).read()
s=s.replace('#include "lr2021_rttof.h"','#include "lr2021_rttof.h"\n#include "lr2021_pram.h"',1)
old="""		if (st == RADIOLIB_ERR_NONE) {
			/* match CustomLR2021's proven config: explicit header, CRC, RX boosted gain */"""
new="""		if (st == RADIOLIB_ERR_NONE) {
			/* Semtech's firmware patch RAM (datasheet 22.3): lost on the reset
			 * begin() just did, never loaded by RadioLib */
			uint16_t pram_ver = 0;
			int16_t ps = lr2021_load_pram(s_lora, &pram_ver);
			if (ps == RADIOLIB_ERR_NONE) printk("radio: PRAM loaded, version %u\\n", pram_ver);
			else printk("radio: PRAM load failed (%d), continuing without it\\n", ps);
			/* match CustomLR2021's proven config: explicit header, CRC, RX boosted gain */"""
assert old in s; s=s.replace(old,new,1); open(t,"w").write(s); print("port ok")
