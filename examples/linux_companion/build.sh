#!/bin/sh
# Builds the stock companion firmware for a Linux host with SDR radios.
# Nothing under src/ or examples/companion_radio/ is modified.
set -e
cd "$(dirname "$0")/../.."
SSL=${OPENSSL_ROOT:-$(brew --prefix openssl 2>/dev/null || echo /usr)}
OUT=${OUT:-examples/linux_companion/build}
mkdir -p "$OUT"

INC="-Isrc -Ilib/ed25519 -Iexamples/linux_companion -Iexamples/linux_companion/host
     -Iexamples/companion_radio -Itest/mocks -I$SSL/include"
# Radio defaults are #ifndef-guarded in MyMesh.h, so a deployment picks its
# region here. These are the EU MeshCore settings this node runs on.
DEF="-DLINUX_PLATFORM=1 -DMESH_DEBUG=${MESH_DEBUG:-1} -DMAX_GROUP_CHANNELS=8 -DMAX_CONTACTS=200
     -DLORA_FREQ=${LORA_FREQ:-869.618} -DLORA_BW=${LORA_BW:-62.5}
     -DLORA_SF=${LORA_SF:-7} -DLORA_CR=${LORA_CR:-5}"
FORCE="-include examples/linux_companion/host/FS.h"
FLAGS="-std=c++17 -O2 -Wno-deprecated-declarations $DEF $FORCE $INC"

SRCS="src/Utils.cpp src/Packet.cpp src/Identity.cpp src/Dispatcher.cpp src/Mesh.cpp
      src/helpers/BaseChatMesh.cpp src/helpers/AdvertDataHelpers.cpp
      src/helpers/TxtDataHelpers.cpp src/helpers/StaticPoolPacketManager.cpp
      src/helpers/IdentityStore.cpp src/helpers/ConfigSerializer.cpp
      src/helpers/ClientACL.cpp src/helpers/RegionMap.cpp
      src/helpers/TransportKeyStore.cpp
      examples/companion_radio/MyMesh.cpp examples/companion_radio/DataStore.cpp
      examples/linux_companion/SdrRadio.cpp examples/linux_companion/TcpSerialInterface.cpp
      examples/linux_companion/main.cpp"

OBJS=""
for s in $SRCS; do
  o="$OUT/$(echo "$s" | tr '/' '_' | sed 's/\.cpp$/.o/')"
  echo "  CXX $s"
  g++ $FLAGS -c "$s" -o "$o"
  OBJS="$OBJS $o"
done
for c in lib/ed25519/*.c; do
  o="$OUT/$(basename "$c" .c).o"
  gcc -O2 -DED25519_NO_SEED=1 -Ilib/ed25519 -c "$c" -o "$o"
  OBJS="$OBJS $o"
done

g++ -o "$OUT/linux_companion" $OBJS -L"$SSL/lib" -lcrypto -lpthread
echo "built $OUT/linux_companion"
