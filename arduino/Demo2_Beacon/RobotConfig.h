// This has generated initial defaults and edit robot_setting.m for this
// Then you're supposed to run configure_robot
// These are placeholders until the measurements sign checks and identification and floor tests all pass.
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H
#include <Arduino.h>
const int RIGHT_IDX=1, LEFT_IDX=0; // motor 1 = LEFT (team config, verify with StepID mode 2)
const int MOTOR_SIGN[2]={-1,1};
const int ENC_SIGN[2]={-1,1};
const int enablePin=4;
const int directionPin[2]={7,8};
const int pwmPin[2]={9,10};
const int encoderA1=3, encoderB1=6;
const int encoderA2=2, encoderB2=5;
const float WHEEL_R=0.074895f;
const float WHEEL_D=0.3683f;
// Wheel radius trim (fractional left-minus-right radius difference).
// Robot curves RIGHT on a straight run -> make this POSITIVE.
// Robot curves LEFT -> make this NEGATIVE.
// From a straight run of length s that ends y to the side (same units):
// WHEEL_R_TRIM_new = WHEEL_R_TRIM_old + 2WHEEL_Dy/s^2 (y>0 = ended right)
// Example: 9 ft run ending 2 in right -> about +0.005
const float WHEEL_R_TRIM=-0.0196f;
const float COUNTS_PER_REV=3200.0f;
const float BATTERY_V=7.8f;
const float K_RHO=0.0566f;
const float K_PHI=0.307f;
const float V_MAX=0.35f;
const float A_MAX=0.4f;
const float W_MAX=1.5f;
const float AW_MAX=2.0f;
const float RHO_DOT_CMD_MAX=0.5f;
const float PHI_DOT_CMD_MAX=2.5f;
const float I_RHO_MAX=0.05f;
const float I_PHI_MAX=0.1f;
const float RHO_TOL=0.002f;
const float PHI_TOL=0.0036f;
const float RATE_TOL_RHO=0.01f;
const float RATE_TOL_PHI=0.03f;
const float STALL_VOLTAGE=1.5f;
const float KPV_RHO=48.6f;
const float KPV_PHI=8.96f;
const float KP_RHO=6.0f;
const float KP_PHI = 6.0f;
const float KI_RHO=3.0f;
const float KI_PHI=3.0f;
const unsigned long TS_US=10000UL;
const unsigned long SETTLE_MS=250UL;
const unsigned long VERIFY_STOP_MS=350UL;
const unsigned long TIMEOUT_MS=5000UL;
const unsigned long START_DELAY_MS=1000UL;
const unsigned long PAUSE_MS=300UL;
const unsigned long MAX_RUN_MS=55000UL;
const unsigned long STALL_MS=2000UL;
const bool MODEL_IDENTIFIED=false;
#endif
