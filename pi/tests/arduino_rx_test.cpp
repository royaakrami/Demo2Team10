// this decodes the bytes the pi sent using the receive code copied out of Demo2_Beacon.ino
// run_all.sh copies that code out and builds this
#include <cstdint>
#include <cstdio>
#include <vector>
#include <fstream>
#include <sstream>
struct FakeWire { std::vector<uint8_t> b; size_t i = 0;
  int read() { return i < b.size() ? b[i++] : -1; } int available() { return (int)(b.size() - i); } } Wire;
#define noInterrupts()
#define interrupts()
struct VisionPacket { uint8_t seq, flags; int16_t angle_cdeg; uint16_t dist_mm; };
#include "rx_extract.inc"     // rxBuf / onI2CReceive from the .ino
int main() {
  std::ifstream f("tests/captured_packets.txt"); std::string line; int n = 0, ok = 0, seen = 0;
  while (std::getline(f, line)) {
    std::istringstream ss(line); int v; Wire.b.clear(); Wire.i = 0;
    while (ss >> v) Wire.b.push_back((uint8_t)v);
    rxNew = false; onI2CReceive((int)Wire.b.size()); n++;
    if (!rxNew) continue;
    VisionPacket p;
#include "decode_extract.inc"  // the decode lines from loop()
    ok++;
    if (p.flags & 1) { seen++; if (seen == 1 || seen % 5 == 0)
      printf("seq %3u flags %u arrow %u angle %+.2f deg dist %.3f ft\n", p.seq, p.flags, (p.flags >> 1) & 3,
             p.angle_cdeg * 0.01, p.dist_mm / 304.8); }
  }
  printf("Arduino decoder: %d packets in, %d accepted, %d bad, %d seen\n", n, ok, (int)rxBad, seen);
  return (ok == n && n > 0) ? 0 : 1;
}
