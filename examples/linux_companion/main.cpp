// linux_companion: the stock MeshCore companion node on a Linux host, with
// a pair of SDRs where the LoRa transceiver would be. Everything above the
// adapters - MyMesh, BaseChatMesh, Mesh, Packet, Identity - is the
// unmodified firmware.
#include <Arduino.h>
#include <Mesh.h>
#include <helpers/ArduinoHelpers.h>
#include <helpers/SimpleMeshTables.h>
#include "host/target.h"
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
SdrRadio radio_driver;

mesh::LocalIdentity radio_new_identity() {
  LinuxRNG rng;
  return mesh::LocalIdentity(&rng);   // /dev/urandom, not radio noise
}

static HostFS g_fs(".");
DataStore store(g_fs, rtc_clock);

StdRNG fast_rng;
SimpleMeshTables tables;
MyMesh the_mesh(radio_driver, fast_rng, rtc_clock, tables, store);

static TcpSerialInterface* g_serial = nullptr;

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
  SdrRadio::Config& g_radio_cfg = radio_driver.config();
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "-p") && i + 1 < argc) port = atoi(argv[++i]);
    else if (!strcmp(argv[i], "-d") && i + 1 < argc) data_dir = argv[++i];
    else if (!strcmp(argv[i], "--rx-device") && i + 1 < argc) g_radio_cfg.rx_device = argv[++i];
    else if (!strcmp(argv[i], "--rx-channels") && i + 1 < argc) g_radio_cfg.rx_channels = argv[++i];
    else if (!strcmp(argv[i], "--rx-sfs") && i + 1 < argc) g_radio_cfg.rx_sfs = argv[++i];
    else if (!strcmp(argv[i], "--rx-ppm") && i + 1 < argc) g_radio_cfg.rx_ppm = atoi(argv[++i]);
    else if (!strcmp(argv[i], "--tx-freq") && i + 1 < argc) g_radio_cfg.tx_freq = (uint32_t)atol(argv[++i]);
    else if (!strcmp(argv[i], "--rx-binary") && i + 1 < argc) g_radio_cfg.rx_binary = argv[++i];
    else if (!strcmp(argv[i], "--tx-binary") && i + 1 < argc) g_radio_cfg.tx_binary = argv[++i];
    else {
      fprintf(stderr,
        "Usage: %s [-p port] [-d data_dir] [--rx-device sub] [--rx-channels list]\n"
        "          [--rx-sfs list] [--rx-ppm n] [--tx-freq hz]\n"
        "          [--rx-binary path] [--tx-binary path]\n", argv[0]);
      return 1;
    }
  }

  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);
  signal(SIGPIPE, SIG_IGN);

  board.begin();
  radio_driver.begin();
  fast_rng.begin((long)time(NULL));

  the_mesh.begin(false /* no display */);

  TcpSerialInterface serial(port);
  g_serial = &serial;
  if (!serial.start()) return 1;
  serial.enable();
  the_mesh.startInterface(serial);

  fprintf(stderr, "[linux_companion] %s on TCP %d, data dir %s\n",
          FIRMWARE_VERSION, port, data_dir.c_str());

  while (true) {
    the_mesh.loop();
    sensors.loop();
    usleep(1000);          // 1 ms; the mesh polls the radio each pass
  }
  return 0;
}
