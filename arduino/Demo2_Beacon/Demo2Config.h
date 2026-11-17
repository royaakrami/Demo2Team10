// these are the demo 2 settings, the robot geometry and gains are still in RobotConfig.h
#ifndef DEMO2_CONFIG_H
#define DEMO2_CONFIG_H
#include <Arduino.h>

// 0 = use the jumper on A3 (open = type 1, jumped to GND = type 2), 1 or 2 = always that type
#define TRIAL_OVERRIDE 0
const int TRIAL_PIN = A3;      // free on the MC33926 shield
const int LED_PIN   = 13;      // on while the Pi reports the marker in view

// the wheel midpoint has to end within 1.5 ft of the beacon, so we aim a little inside that
const float STOP_DIST_FT = 1.25f;
// how far the camera lens is in front of the wheel axle, the camera sits over the axle so its 0
// if the camera moves forward again, put the gap here in ft (10 cm = 0.328f)
const float CAM_FWD_OFFSET_FT = 0.0f;

// search turns in steps and looks, the step has to be smaller than the camera view (about 50 deg)
const float SEARCH_STEP_DEG = 35.0f;
const int   SEARCH_DIR      = +1;       // +1 = CCW (left), -1 = CW (right)
const unsigned long SEARCH_LOOK_MS = 700;  // stand still this long after each step, long enough for 2 frames at ~4 fps
const uint8_t FOUND_FRAMES = 2;         // back-to-back "seen" packets needed to lock on

const float VISION_LATENCY_S = 0.25f;   // frame age when it reaches the Arduino, our camera runs at ~4 fps
const float VISION_BLEND     = 0.4f;    // share of each new fix that moves the goal (1 = take it fully)
const float VISION_CUTOFF_FT = 0.5f;    // closer than this to the stop point: ignore vision, finish on encoders
const float VISION_JUMP_FT   = 1.0f;    // a single fix that moves the goal more than this is ignored
const float VISION_JUMP_DEG  = 15.0f;
const uint8_t VISION_JUMP_ACCEPT = 3;   // unless this many in a row agree
const float BEACON_MIN_FT = 0.3f;       // fixes outside this range are thrown away
const float BEACON_MAX_FT = 9.0f;
const unsigned long PI_TIMEOUT_MS = 400;     // no packet for this long = marker not seen
const unsigned long PI_WAIT_MAX_MS = 20000;  // start searching even if the Pi never shows up

// same speed limits as demo 1, raise them once the runs are reliable
const float APPROACH_V_MAX = 0.35f;     // m/s
const float APPROACH_A_MAX = 0.40f;     // m/s^2
const float TURN_W_MAX     = 1.5f;      // rad/s
const float TURN_AW_MAX    = 2.0f;      // rad/s^2
const float ALIGN_DONE_DEG = 3.0f;      // start driving once the heading error is below this
const float FINAL_TURN_DEG = 90.0f;

// arrival checks are loose on purpose, and a timeout moves on instead of faulting
const float ARRIVE_RHO_TOL = 0.01f;     // m
const float ARRIVE_PHI_TOL = 0.035f;    // rad (2 deg)
const unsigned long ARRIVE_HOLD_MS    = 120;
const unsigned long ARRIVE_TIMEOUT_MS = 1500;
const unsigned long START_DELAY_MS_D2 = 500;

const uint8_t I2C_ADDR = 8;             // same address as the Mini Project
#endif
