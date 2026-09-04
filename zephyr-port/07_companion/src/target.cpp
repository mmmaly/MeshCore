#include "target.h"
#include "zephyr_radiolib_hal.h"
/* Chip quirks (header-CRC standby, RX max-length re-assert) and the mesh wrapper
 * (standby-before-re-arm, RSSI hooks) are shared with the bare-metal variant via
 * CustomLR2021 / CustomLR2021Wrapper — single source for the LR2021 fixes. */
#include <helpers/radiolib/CustomLR2021Wrapper.h>
#include <zephyr/sys/reboot.h>

static ZephyrHal hal;

static Module s_mod(&hal, LR_PIN_NSS, LR_PIN_DIO1, LR_PIN_RESET, LR_PIN_BUSY);
static CustomLR2021 s_lora(&s_mod);

ZBoard board;   /* defined before the radio wrapper that references it */

static CustomLR2021Wrapper s_radio(s_lora, board);

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


RadioLibWrapper &radio_driver = s_radio;
VolatileRTCClock rtc_clock;
SensorManager    sensors;

mesh::LocalIdentity radio_new_identity()
{
	RadioNoiseListener rng(s_lora);
	return mesh::LocalIdentity(&rng);   /* new identity from LoRa RSSI noise */
}

void ZBoard::reboot() { sys_reboot(SYS_REBOOT_COLD); }

#define MC_HF_CUTOFF_MHZ  1500.0f

static float  s_freq    = LORA_FREQ;
static int8_t s_req_dbm = LORA_TX_POWER;   /* last app-requested power, pre-clamp */

static int8_t clamp_tx_for_band(float freq, int8_t dbm)
{
	if (freq > MC_HF_CUTOFF_MHZ)                       /* 2.4 GHz HF PA: -19..+12 */
		return dbm > 12 ? 12 : (dbm < -19 ? -19 : dbm);
	return dbm > 22 ? 22 : (dbm < -9 ? -9 : dbm);     /* sub-GHz LF PA: -9..+22 */
}


static bool radio_bringup(float freq, float bw, uint8_t sf, uint8_t cr)
{
	for (int attempt = 0; attempt < 8; attempt++) {

		int16_t st = s_lora.begin(freq, bw, sf, cr,
					  RADIOLIB_LR2021_LORA_SYNC_WORD_PRIVATE,
					  clamp_tx_for_band(freq, s_req_dbm), 16, /*tcxoVoltage=*/0.0f);
		if (st == RADIOLIB_ERR_NONE) {
			/* match CustomLR2021's proven config: explicit header, CRC, RX boosted gain */
			s_lora.explicitHeader();
			s_lora.setCRC(2);
			s_lora.setRxBoostedGainMode(LR2021_RX_BOOST_LEVEL);
			int16_t rx = s_lora.startReceive();      /* the operation that fails on a bad boot */
			if (rx == RADIOLIB_ERR_NONE) {
				s_lora.standby();                /* idle; the wrapper arms RX in its loop */
				s_freq = freq;
				if (attempt) printk("radio: RX ok after %d retr%s\n",
						    attempt, attempt == 1 ? "y" : "ies");
				return true;
			}
			printk("radio: begin ok but RX arm failed (%d), resetting\n", rx);
		} else {
			printk("radio: begin failed (%d), resetting\n", st);
		}
		s_lora.reset();        /* full chip reset, then retry the whole bring-up */
		k_msleep(50);
	}
	return false;
}

bool radio_init()
{
	s_lora.setIrqDio(LR2021_IRQ_DIO);
	s_lora.setPaTable(wio_pa_lf, false);   /* before begin(): setOutputPower() reads it */
	s_freq = LORA_FREQ;
	s_req_dbm = LORA_TX_POWER;
	if (!radio_bringup(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR)) return false;
	s_radio.begin();
	return true;
}

uint32_t radio_get_rng_seed() { return s_lora.random(0x7FFFFFFF); }

void radio_set_params(float freq, float bw, uint8_t sf, uint8_t cr)
{

	bool band_changed = (freq > MC_HF_CUTOFF_MHZ) != (s_freq > MC_HF_CUTOFF_MHZ);
	s_lora.standby();
	int16_t st = s_lora.setFrequency(freq);     /* recalibrates the front-end for the band */
	if (st == RADIOLIB_ERR_NONE) st = s_lora.setBandwidth(bw);
	if (st == RADIOLIB_ERR_NONE) st = s_lora.setSpreadingFactor(sf);
	if (st == RADIOLIB_ERR_NONE) st = s_lora.setCodingRate(cr);
	s_freq = freq;
	if (band_changed) s_lora.setOutputPower(clamp_tx_for_band(freq, s_req_dbm));
	s_radio.begin();   /* reset wrapper -> dispatcher re-arms RX (startReceive) on the new config */
	if (st != RADIOLIB_ERR_NONE) {
		printk("radio_set_params: apply error %d for %u kHz bw=%u kHz sf%u cr%u\n",
		       st, (unsigned)(freq * 1000.0f), (unsigned)(bw * 1000.0f), sf, cr);
	} else {
		printk("radio_set_params: now on %u kHz bw=%u kHz sf%u cr%u\n",
		       (unsigned)(freq * 1000.0f), (unsigned)(bw * 1000.0f), sf, cr);
	}
}

void radio_set_tx_power(int8_t dbm)
{
	s_req_dbm = dbm;
	s_lora.setOutputPower(clamp_tx_for_band(s_freq, dbm));
}
