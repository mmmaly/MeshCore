// linux_companion: the stock MeshCore companion node on a Linux host, with
// a pair of SDRs where the LoRa transceiver would be. Everything above the
// adapters - MyMesh, BaseChatMesh, Mesh, Packet, Identity - is the
// unmodified firmware.
#include <Arduino.h>
#include <Mesh.h>
#include <helpers/ArduinoHelpers.h>
#include <helpers/SimpleMeshTables.h>
#include "host/target.h"
#include "SdrRadio.h"
#include "SerialRadio.h"
#include "MyMesh.h"
#include "TcpSerialInterface.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <string>

/* GLOBAL OBJECTS the companion sources expect from a variant */
LinuxBoard board;
LinuxRTCClock rtc_clock;
SensorManager sensors;
static SdrRadio g_sdr_radio;        // RTL-SDR rx + HackRF tx (default)
static SerialRadio g_serial_radio;  // LR2021 EVK on a serial port (--serial)
RadioProxy radio_driver;

mesh::LocalIdentity radio_new_identity() {
  LinuxRNG rng;
  return mesh::LocalIdentity(&rng);   // /dev/urandom, not radio noise
}

static HostFS g_fs(".");
DataStore store(g_fs, rtc_clock);

StdRNG fast_rng;
SimpleMeshTables tables;

static TcpSerialInterface* g_serial = nullptr;

// Stock MyMesh plus LR2021 time-of-flight ranging (RangingControl.h): a private
// app frame 0xF0 asks us to range a contact; a private control packet asks a
// peer to answer as subordinate. Everything else is the unmodified companion.
class RangingMesh : public MyMesh {
public:
  using MyMesh::MyMesh;
  RangingRequest defaults{{0, 0, 0, 0}, 2450000000u, 500000, 8, 10, 20438};

  void onControlDataRecv(mesh::Packet* packet) override {
    RangingRequest req;
    if (rangingDecodeRequest(packet->payload, packet->payload_len, req)) {
      if (memcmp(req.peer, self_id.pub_key, 4) != 0) return;     // for someone else
      fprintf(stderr, "[ranging] request from a neighbour: %u Hz bw %u sf %u, %u exchanges - answering as subordinate\n",
              req.freq_hz, req.bw_hz, req.sf, req.count);
      radio_driver.rangeSubordinate(req, rangingAddrFromPubKey(self_id.pub_key));   // blocks for the window
      return;
    }
    MyMesh::onControlDataRecv(packet);
  }

  void loop() {                       // not virtual upstream; main() calls it on this type
    MyMesh::loop();
    uint8_t frame[64];
    size_t n = g_serial ? g_serial->takePrivateFrame(frame, sizeof(frame)) : 0;
    if (n >= 2 + 32 && frame[0] == RANGING_CMD_CODE) {
      RangingResult res;
      rangeContact(&frame[1], n >= 34 ? frame[33] : defaults.count, res);
      uint8_t out[16];
      g_serial->writeFrame(out, rangingEncodeResult(res, out));
    }
  }

  // Requester side: tell the peer, wait for it to arm, run the exchanges.
  void rangeContact(const uint8_t* peer_pubkey, uint8_t count, RangingResult& res) {
    RangingRequest req = defaults;
    memcpy(req.peer, peer_pubkey, 4);
    if (count) req.count = count;
    uint8_t payload[RANGING_REQ_LEN];
    rangingEncodeRequest(req, payload);
    mesh::Packet* pkt = createControlData(payload, RANGING_REQ_LEN);
    if (!pkt) { res.status = 3; return; }
    pkt->header &= ~PH_ROUTE_MASK;
    pkt->header |= ROUTE_TYPE_DIRECT;
    pkt->path_len = 0;                                  // zero hop, like sendZeroHop()
    uint8_t raw[MAX_TRANS_UNIT + 8];
    int len = pkt->writeTo(raw);
    releasePacket(pkt);
    fprintf(stderr, "[ranging] asking %02x%02x%02x%02x for %u exchanges at %u Hz\n", req.peer[0], req.peer[1], req.peer[2], req.peer[3], req.count, req.freq_hz);
    if (!radio_driver.startSendRaw(raw, len)) { res.status = 3; return; }   // synchronous on the modem
    usleep(RANGING_SETUP_MS * 1000);
    radio_driver.rangeManager(req, rangingAddrFromPubKey(peer_pubkey), res);
    fprintf(stderr, "[ranging] result: status %u, %u/%u valid, median %.2f m (min %.2f max %.2f)\n",
            res.status, res.valid, res.count, res.median_cm / 100.0, res.min_cm / 100.0, res.max_cm / 100.0);
  }
};
RangingMesh the_rmesh(radio_driver, fast_rng, rtc_clock, tables, store);   // (MyMesh.h's `extern MyMesh the_mesh` is for the Arduino main only)

static void on_signal(int) {
  // Joining threads from a signal handler is neither async-signal-safe nor
  // reliable (a blocking read() need not wake), and it hung shutdown hard
  // enough to need SIGKILL. Just leave: the OS reaps the sockets, and the
  // lora_rx child carries PR_SET_PDEATHSIG so it dies with us.
  _exit(0);
}

int main(int argc, char* argv[]) {
  int port = 5000;
  std::string data_dir = ".";
  SdrRadio::Config& g_radio_cfg = g_sdr_radio.config();
  SerialRadio::Config& g_serial_cfg = g_serial_radio.config();
  bool use_serial = false;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "-p") && i + 1 < argc) port = atoi(argv[++i]);
    else if (!strcmp(argv[i], "-d") && i + 1 < argc) data_dir = argv[++i];
    else if (!strcmp(argv[i], "--rx-device") && i + 1 < argc) g_radio_cfg.rx_device = argv[++i];
    else if (!strcmp(argv[i], "--rx-channels") && i + 1 < argc) g_radio_cfg.rx_channels = argv[++i];
    else if (!strcmp(argv[i], "--rx-sfs") && i + 1 < argc) g_radio_cfg.rx_sfs = argv[++i];
    else if (!strcmp(argv[i], "--rx-ppm") && i + 1 < argc) g_radio_cfg.rx_ppm = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--rx-agc")) g_radio_cfg.rx_agc = true;   // -G -T: proven on the Blog V4, deaf on the UHIDIR
    else if (!strcmp(argv[i], "--tx-freq") && i + 1 < argc) g_radio_cfg.tx_freq = (uint32_t)atol(argv[++i]);
    else if (!strcmp(argv[i], "--rx-binary") && i + 1 < argc) g_radio_cfg.rx_binary = argv[++i];
    else if (!strcmp(argv[i], "--tx-binary") && i + 1 < argc) g_radio_cfg.tx_binary = argv[++i];
    else if (!strcmp(argv[i], "--serial") && i + 1 < argc) { use_serial = true; g_serial_cfg.device = argv[++i]; }
    else if (!strcmp(argv[i], "--serial-baud") && i + 1 < argc) g_serial_cfg.baud = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--side-sfs") && i + 1 < argc) g_serial_cfg.side_sfs = argv[++i];
    else if (!strcmp(argv[i], "--tx-power") && i + 1 < argc) g_serial_cfg.tx_power = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--rx-boost") && i + 1 < argc) g_serial_cfg.rx_boost = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--range-freq") && i + 1 < argc) the_rmesh.defaults.freq_hz = (uint32_t)strtoul(argv[++i], nullptr, 10);
    else if (!strcmp(argv[i], "--range-bw") && i + 1 < argc) the_rmesh.defaults.bw_hz = (uint32_t)strtoul(argv[++i], nullptr, 10);
    else if (!strcmp(argv[i], "--range-sf") && i + 1 < argc) the_rmesh.defaults.sf = (uint8_t)atoi(argv[++i]);
    else if (!strcmp(argv[i], "--range-delay") && i + 1 < argc) the_rmesh.defaults.delay = (uint32_t)strtoul(argv[++i], nullptr, 10);
    else {
      fprintf(stderr,
        "Usage: %s [-p port] [-d data_dir]\n"
        "  SDR radios (default):  [--rx-device sub] [--rx-channels list] [--rx-sfs list]\n"
        "                         [--rx-ppm n] [--rx-agc] [--tx-freq hz] [--rx-binary path] [--tx-binary path]\n"
        "  LR2021 EVK (lr2021_serial firmware): --serial /dev/ttyACM0 [--serial-baud n]\n"
        "                         [--side-sfs 8,9] [--tx-power dbm] [--rx-boost 0..7]\n"
        "  ranging (LR2021):      [--range-freq hz] [--range-bw hz] [--range-sf n] [--range-delay n]\n", argv[0]);
      return 1;
    }
  }
  radio_driver.select(use_serial ? (HostRadio*)&g_serial_radio : (HostRadio*)&g_sdr_radio);

  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);
  signal(SIGPIPE, SIG_IGN);

  board.begin();
  radio_driver.begin();
  fast_rng.begin((long)time(NULL));

  store.begin();                       // must precede the_rmesh.begin(): it
                                       // creates /identity and loads prefs
  the_rmesh.begin(false /* no display */);

  TcpSerialInterface serial(port);
  g_serial = &serial;
  if (!serial.start()) return 1;
  serial.enable();
  the_rmesh.startInterface(serial);

  fprintf(stderr, "[linux_companion] %s on TCP %d, data dir %s\n",
          FIRMWARE_VERSION, port, data_dir.c_str());

  while (true) {
    the_rmesh.loop();
    sensors.loop();
    usleep(1000);          // 1 ms; the mesh polls the radio each pass
  }
  return 0;
}
