// this is the demo 2 arduino code, it finds the beacon, drives up to it and stops 1.25 ft away
// for type 2 trials it then turns 90 deg toward the arrow
// WAIT until the pi sends packets, SEARCH turns in 35 deg steps until the marker is seen,
// ALIGN turns to face it, APPROACH drives in, FINAL_TURN does the 90 deg turn, DONE prints the result
// each vision fix moves the goal a little, and the last 0.5 ft is done on the encoders only
// the control loop is the same as demo 1, and RobotConfig.h / RobotIO.h are unchanged
// jumper A3 to GND = type 2, open = type 1 (read at reset)
// test without the pi: serial monitor 115200, type  v 20 5 R  (fake marker) or  s  (status)
#include <Wire.h>
#include "RobotConfig.h"
#include "RobotIO.h"
#include "Demo2Config.h"
#include "Demo2Types.h"

const float FT_TO_M = 0.3048f, M_TO_IN = 39.37007874f, DEG2RAD = PI / 180.0f;

// i2c from the pi, filled in the interrupt and used in loop
volatile uint8_t rxBuf[6];
volatile bool rxNew = false;
volatile uint16_t rxBad = 0;
volatile uint8_t txStatus = 0;

// the pi sends an offset byte and then 7 data bytes
void onI2CReceive(int n) {
  if (n < 1) return;               // i2cdetect probe
  Wire.read();                     // offset byte, not used
  if (n - 1 != 7) { while (Wire.available()) Wire.read(); rxBad++; return; }
  uint8_t b[7];
  for (int i = 0; i < 7; i++) b[i] = Wire.read();
  uint8_t x = 0;
  for (int i = 0; i < 6; i++) x ^= b[i];
  if (x != b[6]) { rxBad++; return; }   // corrupted packet, drop it
  for (int i = 0; i < 6; i++) rxBuf[i] = b[i];
  rxNew = true;
}
// the pi reads one byte back: bits 0-3 state, bits 4-5 turn (1 left 2 right), bit 6 type 2
void onI2CRequest() { Wire.write((uint8_t)txStatus); }

State state = WAIT;
int trialType = 1;
uint32_t t0 = 0, state_ms = 0, run_ms = 0, lastUs = 0, lastLogMs = 0;
const char *faultReason = "NONE";
bool resultPrinted = false;

Ramp rr = {0, 0}, pr = {0, 0};      // rho and phi references
float rhoGoal = 0, phiGoal = 0;     // goals from vision
float rhoHold = 0, phiFinal = 0, searchGoal = 0;
bool haveFix = false;
float I_rho = 0, I_phi = 0;

// vision bookkeeping
bool piAlive = false;
uint32_t lastPacketMs = 0;
uint8_t seenStreak = 0, jumpCount = 0;
uint16_t packets = 0, votesL = 0, votesR = 0;
int8_t lastArrow = 0, turnDir = 0;

// search / arrival timers
bool looking = false;
uint32_t lookStart = 0;
bool holdActive = false, refDone = false;
uint32_t holdStart = 0, refDoneMs = 0;

// stall detection (same idea as Demo 1)
long activityR = 0, activityL = 0;
uint32_t lastActivityR = 0, lastActivityL = 0;

void stepRamp(Ramp &r, float goal, float vmax, float amax, float dt) {
  float e = goal - r.ref;
  float vstop = sqrtf(2.0f * amax * fabs(e));          // fastest speed that can still stop on the goal
  float vdes = clipValue(e >= 0 ? vstop : -vstop, -vmax, vmax);
  r.vel += clipValue(vdes - r.vel, -amax * dt, amax * dt);
  float step = r.vel * dt;
  if ((e > 0 && step >= e) || (e < 0 && step <= e) || (e == 0 && fabs(r.vel) <= amax * dt)) {
    r.ref = goal; r.vel = 0;                           // land on the goal doesn't step past it
  } else {
    r.ref += step;
  }
}
bool rampAt(const Ramp &r, float goal, float tol) { return fabs(goal - r.ref) <= tol && r.vel == 0; }

void enterState(State s) {
  state = s; state_ms = millis();
  holdActive = false; refDone = false; looking = false;
}
void fail(const char *why) { faultReason = why; enterState(FAULT); stopMotors(); digitalWrite(enablePin, LOW); }

// true when the robot is at the goal and still, or the reference has sat there long enough to move on
bool arrived(float rhoG, float phiG, uint32_t now) {
  bool refAt = fabs(rhoG - rr.ref) < 1e-3f && fabs(phiG - pr.ref) < 1e-3f && rr.vel == 0 && pr.vel == 0;
  if (refAt && !refDone) { refDone = true; refDoneMs = now; }
  if (!refAt) refDone = false;
  bool close = fabs(rhoG - pose.rho) <= ARRIVE_RHO_TOL && fabs(phiG - pose.phi) <= ARRIVE_PHI_TOL &&
               fabs(pose.rhod) <= RATE_TOL_RHO * 3 && fabs(pose.phid) <= RATE_TOL_PHI * 3;
  if (refAt && close) {
    if (!holdActive) { holdActive = true; holdStart = now; }
    if (now - holdStart >= ARRIVE_HOLD_MS) return true;
  } else holdActive = false;
  return refDone && (now - refDoneMs >= ARRIVE_TIMEOUT_MS);
}

int8_t decideTurn() {                 // +1 = left (CCW), -1 = right (CW)
  if (votesL > votesR) return +1;
  if (votesR > votesL) return -1;
  return lastArrow != 0 ? lastArrow : +1;
}

void handlePacket(const VisionPacket &p, uint32_t now) {
  piAlive = true; lastPacketMs = now; packets++;
  bool seen = p.flags & 0x01;
  uint8_t arrow = (p.flags >> 1) & 0x03;
  if (!seen) { seenStreak = 0; return; }

  // angle and distance from the wheel midpoint, the camera can sit ahead of it
  float a = p.angle_cdeg * 0.01f * DEG2RAD, d = p.dist_mm * 0.001f;
  float c = CAM_FWD_OFFSET_FT * FT_TO_M;
  float fx = c + d * cos(a), fy = d * sin(a);
  float bearing = atan2(fy, fx), range = sqrt(fx * fx + fy * fy);
  if (range < BEACON_MIN_FT * FT_TO_M || range > BEACON_MAX_FT * FT_TO_M) { seenStreak = 0; return; }
  if (seenStreak < 255) seenStreak++;

  if (state != FINAL_TURN && state != DONE) {
    if (arrow == 1) { votesL++; lastArrow = +1; }
    else if (arrow == 2) { votesR++; lastArrow = -1; }
  }
  if (state == FINAL_TURN || state == DONE || state == FAULT) return;
  if (state == APPROACH && (rhoGoal - pose.rho) < VISION_CUTOFF_FT * FT_TO_M) return;

  // where the robot was when the frame was taken
  float phiAt = pose.phi - pose.phid * VISION_LATENCY_S;
  float rhoAt = pose.rho - pose.rhod * VISION_LATENCY_S;
  float phiMeas = phiAt + bearing;
  float rhoMeas = rhoAt + range - STOP_DIST_FT * FT_TO_M;
  if (rhoMeas < pose.rho) rhoMeas = pose.rho;        // already inside the stop distance: don't back up

  if (!haveFix) {
    if (seenStreak >= FOUND_FRAMES) { phiGoal = phiMeas; rhoGoal = rhoMeas; haveFix = true; jumpCount = 0; }
    return;
  }
  bool jump = fabs(phiMeas - phiGoal) > VISION_JUMP_DEG * DEG2RAD ||
              fabs(rhoMeas - rhoGoal) > VISION_JUMP_FT * FT_TO_M;
  if (jump) {
    if (++jumpCount >= VISION_JUMP_ACCEPT) { phiGoal = phiMeas; rhoGoal = rhoMeas; jumpCount = 0; }
    return;
  }
  jumpCount = 0;
  phiGoal += VISION_BLEND * (phiMeas - phiGoal);
  rhoGoal += VISION_BLEND * (rhoMeas - rhoGoal);
}

// serial test commands
char cmdBuf[40];
uint8_t cmdLen = 0;
void printStatus() {
  Serial.print(F("# STATUS state=")); Serial.print((int)state);
  Serial.print(F(" type=")); Serial.print(trialType);
  Serial.print(F(" pi=")); Serial.print(piAlive);
  Serial.print(F(" packets=")); Serial.print(packets);
  Serial.print(F(" bad=")); Serial.print(rxBad);
  Serial.print(F(" fix=")); Serial.print(haveFix);
  Serial.print(F(" rhoGoal_in=")); Serial.print(rhoGoal * M_TO_IN, 2);
  Serial.print(F(" phiGoal_deg=")); Serial.print(phiGoal / DEG2RAD, 2);
  Serial.print(F(" votesL=")); Serial.print(votesL);
  Serial.print(F(" votesR=")); Serial.println(votesR);
}
void readSerial(uint32_t now) {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch != '\n' && ch != '\r') { if (cmdLen < sizeof(cmdBuf) - 1) cmdBuf[cmdLen++] = ch; continue; }
    if (cmdLen == 0) continue;
    cmdBuf[cmdLen] = 0; cmdLen = 0;
    char *tok = strtok(cmdBuf, " ");
    if (!tok) continue;
    if (tok[0] == 's') { printStatus(); continue; }
    if (tok[0] == 'v') {
      char *sa = strtok(NULL, " "), *sd = strtok(NULL, " "), *sr = strtok(NULL, " ");
      if (!sa || !sd) { Serial.println(F("# use: v <angle_deg> <dist_ft> [L|R]")); continue; }
      VisionPacket p;
      p.seq = 0;
      p.flags = 0x01;
      if (sr && (sr[0] == 'L' || sr[0] == 'l')) p.flags |= (1 << 1);
      if (sr && (sr[0] == 'R' || sr[0] == 'r')) p.flags |= (2 << 1);
      p.angle_cdeg = (int16_t)(atof(sa) * 100.0f);
      p.dist_mm = (uint16_t)(atof(sd) * 304.8f);
      for (uint8_t i = 0; i < FOUND_FRAMES; i++) handlePacket(p, now);  // enough to lock on, then silence
      Serial.println(F("# injected vision fix"));
    }
  }
}

void setup() {
  initRobot();
  Serial.begin(115200);
  printConfig();
  pinMode(TRIAL_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
#if TRIAL_OVERRIDE == 1 || TRIAL_OVERRIDE == 2
  trialType = TRIAL_OVERRIDE;
#else
  delay(5);
  trialType = digitalRead(TRIAL_PIN) == LOW ? 2 : 1;
#endif
  Wire.begin(I2C_ADDR);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest);
  Serial.print(F("# DEMO2 trial_type=")); Serial.print(trialType);
  Serial.print(F(" stop_ft=")); Serial.println(STOP_DIST_FT, 2);
  Serial.println(F("# t\tstate\trho_ref\trho\tphi_ref\tphi\trho_goal\tphi_goal\tseen\tVR\tVL"));
  Serial.println(F("# STATES 0=WAIT 1=SEARCH 2=ALIGN 3=APPROACH 4=FINAL_TURN 5=DONE 6=FAULT"));
  if (!configValid()) fail("INVALID_CONFIG");
  t0 = millis(); run_ms = t0; lastLogMs = t0;
  lastActivityR = t0; lastActivityL = t0;
  enterState(state == FAULT ? FAULT : WAIT);
  lastUs = micros();
}

void loop() {
  uint32_t nowUs = micros(), elapsedUs = nowUs - lastUs;
  if (elapsedUs < TS_US) return;
  lastUs = nowUs;
  // Use actual sample duration; do not run short catch-up iterations.
  float dt = elapsedUs * 1e-6f;
  uint32_t now = millis();
  bool moving = state == SEARCH || state == ALIGN || state == APPROACH || state == FINAL_TURN;
  if (elapsedUs > 3 * TS_US && moving) fail("CONTROL_TIMING_OVERRUN");
  readPose(dt);

  // new packet from the pi
  if (rxNew) {
    VisionPacket p;
    noInterrupts();
    p.seq = rxBuf[0]; p.flags = rxBuf[1];
    p.angle_cdeg = (int16_t)((uint16_t)rxBuf[2] | ((uint16_t)rxBuf[3] << 8));
    p.dist_mm = (uint16_t)rxBuf[4] | ((uint16_t)rxBuf[5] << 8);
    rxNew = false;
    interrupts();
    handlePacket(p, now);
  }
  if (piAlive && now - lastPacketMs > PI_TIMEOUT_MS) seenStreak = 0;   // Pi went quiet
  digitalWrite(LED_PIN, seenStreak > 0 ? HIGH : LOW);
  readSerial(now);

  if (moving && now - run_ms >= MAX_RUN_MS) fail("RUN_TIME_LIMIT");
  uint32_t inState = now - state_ms;
  float vmaxR = APPROACH_V_MAX, amaxR = APPROACH_A_MAX;

  switch (state) {
    case WAIT:
      // keep the references on the robot so nothing jumps when we start
      rr.ref = pose.rho; rr.vel = 0; pr.ref = pose.phi; pr.vel = 0;
      if (inState >= START_DELAY_MS_D2 && (piAlive || inState >= PI_WAIT_MAX_MS)) {
        run_ms = now;
        rhoHold = pose.rho;
        if (haveFix) enterState(ALIGN);
        else { searchGoal = pose.phi + SEARCH_DIR * SEARCH_STEP_DEG * DEG2RAD; enterState(SEARCH); }
      }
      break;

    case SEARCH:
      if (haveFix) { rhoHold = rr.ref; enterState(ALIGN); break; }
      stepRamp(pr, searchGoal, TURN_W_MAX, TURN_AW_MAX, dt);
      stepRamp(rr, rhoHold, vmaxR, amaxR, dt);
      if (rampAt(pr, searchGoal, 1e-4f)) {
        if (!looking) { looking = true; lookStart = now; }
        else if (now - lookStart >= SEARCH_LOOK_MS) {
          searchGoal += SEARCH_DIR * SEARCH_STEP_DEG * DEG2RAD;
          looking = false;
        }
      }
      break;

    case ALIGN:
      stepRamp(pr, phiGoal, TURN_W_MAX, TURN_AW_MAX, dt);
      stepRamp(rr, rhoHold, vmaxR, amaxR, dt);
      if ((fabs(phiGoal - pr.ref) < 0.5f * DEG2RAD && fabs(phiGoal - pose.phi) < ALIGN_DONE_DEG * DEG2RAD) ||
          inState > 4000) {
        I_rho = 0;
        enterState(APPROACH);
      }
      break;

    case APPROACH:
      stepRamp(pr, phiGoal, TURN_W_MAX, TURN_AW_MAX, dt);
      stepRamp(rr, rhoGoal, vmaxR, amaxR, dt);
      if (arrived(rhoGoal, phiGoal, now)) {
        if (trialType == 2) {
          turnDir = decideTurn();
          phiFinal = phiGoal + turnDir * FINAL_TURN_DEG * DEG2RAD;
          rhoHold = rhoGoal;
          enterState(FINAL_TURN);
        } else enterState(DONE);
      }
      break;

    case FINAL_TURN:
      stepRamp(pr, phiFinal, TURN_W_MAX, TURN_AW_MAX, dt);
      stepRamp(rr, rhoHold, vmaxR, amaxR, dt);
      if (arrived(rhoHold, phiFinal, now)) enterState(DONE);
      break;

    case DONE: case FAULT: break;
  }

  // control, same cascade as demo 1
  moving = state == SEARCH || state == ALIGN || state == APPROACH || state == FINAL_TURN;
  float vr = 0, vl = 0;
  if (moving) {
    float er = rr.ref - pose.rho, ep = pr.ref - pose.phi;
    float rd0 = rr.vel + KP_RHO * er + KI_RHO * I_rho;
    float pd0 = pr.vel + KP_PHI * ep + KI_PHI * I_phi;
    float rd = clipValue(rd0, -RHO_DOT_CMD_MAX, RHO_DOT_CMD_MAX);
    float pd = clipValue(pd0, -PHI_DOT_CMD_MAX, PHI_DOT_CMD_MAX);
    // Inner loops: desired speed / identified DC gain + proportional correction.
    // vb is the SUM of the motor voltages; dv is RIGHT minus LEFT.
    float vb = rd / K_RHO + KPV_RHO * (rd - pose.rhod);
    float dv = pd / K_PHI + KPV_PHI * (pd - pose.phid);
    vr = (vb + dv) * 0.5f; vl = (vb - dv) * 0.5f;
    // Scale both channels together if either exceeds available battery voltage.
    float largest = fmax(fabs(vr), fabs(vl));
    bool sat = largest > BATTERY_V;
    if (sat) { vr *= BATTERY_V / largest; vl *= BATTERY_V / largest; }
    // Conditional integration accounts for BOTH rate limits and motor limits.
    // Always allow an existing integral to unwind toward zero.
    if ((!sat && rd == rd0) || er * I_rho < 0) I_rho = clipValue(I_rho + er * dt, -I_RHO_MAX, I_RHO_MAX);
    if ((!sat && pd == pd0) || ep * I_phi < 0) I_phi = clipValue(I_phi + ep * dt, -I_PHI_MAX, I_PHI_MAX);

    if (pose.cR != activityR) { activityR = pose.cR; lastActivityR = now; }
    if (pose.cL != activityL) { activityL = pose.cL; lastActivityL = now; }
    if (fabs(vr) < STALL_VOLTAGE) lastActivityR = now;
    if (fabs(vl) < STALL_VOLTAGE) lastActivityL = now;
    if (now - lastActivityR > STALL_MS) fail("RIGHT_NO_ENCODER_PROGRESS");
    else if (now - lastActivityL > STALL_MS) fail("LEFT_NO_ENCODER_PROGRESS");
  } else { lastActivityR = now; lastActivityL = now; }
  if (state == FAULT || !moving) { vr = 0; vl = 0; }
  vr = setMotor(RIGHT_IDX, vr);
  vl = setMotor(LEFT_IDX, vl);

  uint8_t dec = turnDir > 0 ? 1 : (turnDir < 0 ? 2 : 0);
  txStatus = (uint8_t)((uint8_t)state | (dec << 4) | (trialType == 2 ? 0x40 : 0));

  // log
  if (now - lastLogMs >= 40 && state != DONE && state != FAULT) {
    lastLogMs = now;
    Serial.print((now - t0) * 0.001f, 3); Serial.print('\t');
    Serial.print((int)state); Serial.print('\t');
    Serial.print(rr.ref, 3); Serial.print('\t'); Serial.print(pose.rho, 3); Serial.print('\t');
    Serial.print(pr.ref, 3); Serial.print('\t'); Serial.print(pose.phi, 3); Serial.print('\t');
    Serial.print(rhoGoal, 3); Serial.print('\t'); Serial.print(phiGoal, 3); Serial.print('\t');
    Serial.print(seenStreak > 0); Serial.print('\t');
    Serial.print(vr, 2); Serial.print('\t'); Serial.println(vl, 2);
  }
  if ((state == DONE || state == FAULT) && !resultPrinted) {
    resultPrinted = true;
    Serial.print(state == DONE ? F("# RESULT SUCCESS") : F("# RESULT FAULT reason="));
    if (state == FAULT) Serial.print(faultReason);
    Serial.print(F(" run_s=")); Serial.print((now - run_ms) * 0.001f, 2);
    Serial.print(F(" rho_in=")); Serial.print(pose.rho * M_TO_IN, 2);
    Serial.print(F(" heading_deg=")); Serial.print(pose.phi / DEG2RAD, 2);
    Serial.print(F(" turn=")); Serial.print(turnDir > 0 ? F("LEFT") : (turnDir < 0 ? F("RIGHT") : F("none")));
    Serial.print(F(" votesL=")); Serial.print(votesL);
    Serial.print(F(" votesR=")); Serial.print(votesR);
    Serial.print(F(" packets=")); Serial.print(packets);
    Serial.print(F(" bad=")); Serial.println(rxBad);
  }
}
