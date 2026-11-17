#!/bin/bash
# runs all the offline tests, no camera or arduino needed. from the pi folder: bash tests/run_all.sh
set -e
cd "$(dirname "$0")/.."
python3 tests/test_offline.py | tail -4
python3 tests/test_loop.py 2>/dev/null | tail -1
INO=../arduino/Demo2_Beacon/Demo2_Beacon.ino
awk '/^volatile uint8_t rxBuf/{p=1} /^void onI2CRequest/{p=0} p' "$INO" > tests/rx_extract.inc
awk '/p.seq = rxBuf\[0\]/{p=1} p{print} /rxNew = false;/{if(p)exit}' "$INO" > tests/decode_extract.inc
if command -v g++ >/dev/null; then
  (cd tests && g++ -std=c++17 -o /tmp/arduino_rx_test arduino_rx_test.cpp) && /tmp/arduino_rx_test | tail -1
else
  echo "(g++ not installed - skipped the Arduino decoder check)"
fi
