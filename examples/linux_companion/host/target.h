#pragma once
// The board-support contract every MeshCore variant provides, for a Linux
// host: no GPIO, no display, no sensors - just the four globals the
// companion sources expect to exist.
#include <helpers/SensorManager.h>
#include "../LinuxPlatform.h"
#include "../SdrRadio.h"

extern LinuxBoard board;
extern SdrRadio radio_driver;
extern LinuxRTCClock rtc_clock;
extern SensorManager sensors;   // base class: every query is a no-op

// Variants build this from radio noise; on a host /dev/urandom is both
// available and better.
mesh::LocalIdentity radio_new_identity();
