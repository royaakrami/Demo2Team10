#ifndef DEMO2_TYPES_H
#define DEMO2_TYPES_H
#include <Arduino.h>
// the types are in a header so arduino sees them before it makes the function prototypes

enum State { WAIT, SEARCH, ALIGN, APPROACH, FINAL_TURN, DONE, FAULT };

// the reference chases a goal that can change while moving, with the same speed and accel limits
struct Ramp { float ref, vel; };

// one vision fix from the pi, see I2C_PROTOCOL.md
struct VisionPacket {
  uint8_t seq;
  uint8_t flags;        // bit0 = marker seen, bits1-2 = arrow (0 none, 1 left, 2 right)
  int16_t angle_cdeg;   // bearing to marker center, 0.01 deg, left positive
  uint16_t dist_mm;     // camera to marker center, mm
};
#endif
