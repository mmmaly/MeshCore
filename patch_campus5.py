# 1) accessors on CustomLR2021 (protected RadioLib cache)
p="src/helpers/radiolib/CustomLR2021.h"; s=open(p).read()
old="  int16_t setRxBoostedGainMode(uint8_t level) {"
new="""  // RadioLib's cached LoRa modulation codes (protected up the hierarchy); the
  // RTToF setup re-programs the same modulation under the ranging packet type.
  uint8_t sfCode() const { return spreadingFactor; }
  uint8_t bwCode() const { return bandwidth; }
  uint8_t crCode() const { return codingRate; }
  uint8_t ldroCode() const { return ldrOptimize; }

  int16_t setRxBoostedGainMode(uint8_t level) {"""
assert old in s; s=s.replace(old,new,1); open(p,"w").write(s)

# 2) target.cpp: ranging entry points that restore the mesh radio afterwards
t="zephyr-port/07_companion/src/target.cpp"; s=open(t).read()
s=s.replace('#include <zephyr/sys/reboot.h>','#include <zephyr/sys/reboot.h>\n#include "lr2021_rttof.h"',1)
s += '''

/* RTToF ranging (RangingControl.h). Both block; afterwards the mesh radio is
 * restored from the given LoRa parameters and the wrapper re-arms RX. */
bool radio_range_subordinate(const RangingRequest& req, uint32_t my_addr, float freq, float bw, uint8_t sf, uint8_t cr)
{
	int answered = 0;
	bool ok = rttof_subordinate(s_lora, req, my_addr, LR2021_IRQ_DIO, rangingWindowMs(req.count), &answered);
	printk("ranging: subordinate window %s, %d responses sent\\n", ok ? "done" : "FAILED", answered);
	radio_set_params(freq, bw, sf, cr);
	radio_set_tx_power(s_req_dbm);
	return ok;
}

bool radio_range_manager(const RangingRequest& req, uint32_t peer_addr, RangingResult& res, float freq, float bw, uint8_t sf, uint8_t cr)
{
	bool ok = rttof_manager(s_lora, req, peer_addr, LR2021_IRQ_DIO, res);
	printk("ranging: manager status %u, %u/%u valid, median %d cm (min %d max %d)\\n",
	       res.status, res.valid, res.count, res.median_cm, res.min_cm, res.max_cm);
	radio_set_params(freq, bw, sf, cr);
	radio_set_tx_power(s_req_dbm);
	return ok;
}
'''
open(t,"w").write(s)
h="zephyr-port/07_companion/src/target.h"; s=open(h).read()
s=s.replace("mesh::LocalIdentity radio_new_identity();","""mesh::LocalIdentity radio_new_identity();

#include "RangingControl.h"
bool radio_range_subordinate(const RangingRequest& req, uint32_t my_addr, float freq, float bw, uint8_t sf, uint8_t cr);
bool radio_range_manager(const RangingRequest& req, uint32_t peer_addr, RangingResult& res, float freq, float bw, uint8_t sf, uint8_t cr);""")
open(h,"w").write(s)

# 3) main.cpp: RangingMesh subclass (responder + requester via a private BLE frame)
m="zephyr-port/07_companion/src/main.cpp"; s=open(m).read()
old='''StdRNG fast_rng;
SimpleMeshTables tables;
DataStore store(InternalFS, rtc_clock);
MyMesh the_mesh(radio_driver, fast_rng, rtc_clock, tables, store);
'''
new='''StdRNG fast_rng;
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
		if (rangingDecodeRequest(packet->payload, packet->payload_len, req)) {
			if (memcmp(req.peer, self_id.pub_key, 4) != 0) return;
			printk("ranging: request from a neighbour (%u Hz bw %u sf %u, %u exchanges)\\n", req.freq_hz, req.bw_hz, req.sf, req.count);
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
'''
assert old in s; s=s.replace(old,new,1)
# private frame poll in the main loop
old2="\twhile (1) {\n\t\tthe_mesh.loop();"
new2='''\twhile (1) {
\t\tthe_mesh.loop();
\t\t{
\t\t\tuint8_t frame[64];
\t\t\tsize_t n = ble.takePrivateFrame(frame, sizeof(frame));
\t\t\tif (n >= 34 && frame[0] == RANGING_CMD_CODE) {
\t\t\t\tRangingResult res;
\t\t\t\tthe_mesh.rangeContact(&frame[1], frame[33], res);
\t\t\t\tuint8_t out[16];
\t\t\t\tble.writeFrame(out, rangingEncodeResult(res, out));
\t\t\t}
\t\t}'''
assert old2 in s; s=s.replace(old2,new2,1)
open(m,"w").write(s); print("patched5")
