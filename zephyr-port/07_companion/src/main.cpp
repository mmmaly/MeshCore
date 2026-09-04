/*
 * Step 4e: the full MeshCore companion on Zephyr. Wires companion_radio's MyMesh +
 * DataStore onto the proven Zephyr seams: radio (target.cpp / RadioLib+ZephyrHal),
 * filesystem (InternalFS / flash_area), and serial = the bonded bt_nus SerialBLEInterface.
 * The phone app pairs (level 4, fixed PIN), then talks the companion protocol over NUS.
 */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include <Arduino.h>
#include <target.h>
#include <helpers/nrf54/InternalFileSystem.h>
#include <helpers/SimpleMeshTables.h>
#include "serial_ble_interface.h"
#include "MyMesh.h"   /* examples/companion_radio (on the include path) */

SerialShim Serial;

StdRNG fast_rng;
SimpleMeshTables tables;
DataStore store(InternalFS, rtc_clock);

/* Stock MyMesh plus LR2021 time-of-flight ranging (RangingControl.h): answer a
 * neighbour's ranging request as subordinate; range a contact on a private app
 * frame 0xF0 (the BLE interface keeps those aside for us). */
class RangingMesh : public MyMesh {
public:
	using MyMesh::MyMesh;
	RangingRequest defaults{{0, 0, 0, 0}, 2450000000u, 500000, 8, 10, 20438};

	void onControlDataRecv(mesh::Packet* packet) override
	{
		RangingRequest req;
		printk("control packet: type %02x len %u\n", packet->payload[0], packet->payload_len);
		if (rangingDecodeRequest(packet->payload, packet->payload_len, req)) {
			if (memcmp(req.peer, self_id.pub_key, 4) != 0) return;
			printk("ranging: request from a neighbour (%u Hz bw %u sf %u, %u exchanges)\n", req.freq_hz, req.bw_hz, req.sf, req.count);
			NodePrefs* p = getNodePrefs();
			radio_range_subordinate(req, rangingAddrFromPubKey(self_id.pub_key), p->freq, p->bw, p->sf, p->cr);
			return;
		}
		MyMesh::onControlDataRecv(packet);
	}

	void rangeContact(const uint8_t* peer_pubkey, uint8_t count, RangingResult& res)
	{
		RangingRequest req = defaults;
		memcpy(req.peer, peer_pubkey, 4);
		if (count) req.count = count;
		uint8_t payload[RANGING_REQ_LEN];
		rangingEncodeRequest(req, payload);
		mesh::Packet* pkt = createControlData(payload, RANGING_REQ_LEN);
		if (!pkt) { res.status = 3; return; }
		pkt->header &= ~PH_ROUTE_MASK;
		pkt->header |= ROUTE_TYPE_DIRECT;
		pkt->path_len = 0;
		uint8_t raw[MAX_TRANS_UNIT + 8];
		int len = pkt->writeTo(raw);
		releasePacket(pkt);
		if (!radio_driver.startSendRaw(raw, len)) { res.status = 3; return; }
		int64_t t0 = k_uptime_get();
		while (!radio_driver.isSendComplete() && k_uptime_get() - t0 < 2000) { k_msleep(2); }
		radio_driver.onSendFinished();
		k_msleep(RANGING_SETUP_MS);
		NodePrefs* p = getNodePrefs();
		radio_range_manager(req, rangingAddrFromPubKey(peer_pubkey), res, p->freq, p->bw, p->sf, p->cr);
	}
};
RangingMesh the_rmesh(radio_driver, fast_rng, rtc_clock, tables, store);
#define the_mesh the_rmesh

int main(void)
{
	printk("\n=== MeshCore companion on Zephyr (board=%s) ===\n", CONFIG_BOARD);

	board.begin();
	if (!radio_init()) { printk("radio_init FAILED\n"); return 0; }
	fast_rng.begin(radio_get_rng_seed());
	if (!InternalFS.begin()) { printk("InternalFS FAILED\n"); return 0; }
	store.begin();
	the_mesh.begin(false);   /* no display (loads prefs from /new_prefs) */

	/* DEBUG: did the node name survive the last power cycle? On a cold boot this should show
	 * the previously-set name and exists=1; if it shows the default name / exists=0 the prefs
	 * file was lost (reformat or never persisted). Paired with the "FS: mounted OK/FAILED" line. */
	printk("PREFS: cold boot -> /new_prefs exists=%d, node_name='%s'\n",
	       InternalFS.exists("/new_prefs"), the_mesh.getNodePrefs()->node_name);

	char name[48];
	snprintf(name, sizeof(name), "%s%s", BLE_NAME_PREFIX, the_mesh.getNodePrefs()->node_name);
#ifndef MC_NO_BLE
	ble.begin(name, the_mesh.getBLEPin());
#else
	printk("BLE stack not started (MC_NO_BLE RF interference test build)\n");
#endif
	the_mesh.startInterface(ble);
	printk("companion up: '%s' pin %lu — connect from the MeshCore app\n",
	       name, (unsigned long)the_mesh.getBLEPin());

	while (1) {
		the_mesh.loop();
		{
			uint8_t frame[64];
			size_t n = ble.takePrivateFrame(frame, sizeof(frame));
			if (n >= 34 && frame[0] == RANGING_CMD_CODE) {
				RangingResult res;
				the_mesh.rangeContact(&frame[1], frame[33], res);
				uint8_t out[16];
				ble.writeFrame(out, rangingEncodeResult(res, out));
			}
		}
		rtc_clock.tick();
		sensors.loop();

		/* A rename from the app (CMD_SET_ADVERT_NAME) only updates prefs; push it to BLE so
		 * the advertised/GAP name follows without a reboot. */
		char cur[48];
		snprintf(cur, sizeof(cur), "%s%s", BLE_NAME_PREFIX, the_mesh.getNodePrefs()->node_name);
		if (strcmp(cur, name) != 0) {
			strcpy(name, cur);
			ble.setDeviceName(name);
			/* DEBUG: confirm the rename reached the FS (savePrefs ran in the CMD handler).
			 * exists=1 means /new_prefs was (re)written; if it's still 0 the save never landed. */
			printk("PREFS: rename -> '%s', /new_prefs exists=%d\n",
			       the_mesh.getNodePrefs()->node_name, InternalFS.exists("/new_prefs"));
		}

		k_msleep(1);   /* snappy serial/radio polling for the app's frame bursts */
	}
	return 0;
}
