// this is a header file for the robot configurations

#ifndef ROBOT_IO_H
#define ROBOT_IO_H
#include "RobotConfig.h"
#include <math.h>
// Same channels/pins and x4-equivalent count scale as the original sketch:
// interrupt on A CHANGE (two events per quadrature cycle), add/subtract TWO.
volatile long encoderCount[2]={0,0};
inline void encoderISR1(){ encoderCount[0] += digitalRead(encoderA1)==digitalRead(encoderB1)?2:-2; }
inline void encoderISR2(){ encoderCount[1] += digitalRead(encoderA2)==digitalRead(encoderB2)?2:-2; }
struct Pose {
  long cR,cL;
  float rho,phi,rhod,phid,x,y;
};
static Pose pose={0,0,0,0,0,0,0,0};
inline float clipValue(float v,float lo,float hi){return v<lo?lo:(v>hi?hi:v);}
inline bool configValid(){
  return WHEEL_R>0 && WHEEL_D>0 && COUNTS_PER_REV>0 && BATTERY_V>0 &&
    RIGHT_IDX>=0 && RIGHT_IDX<2 && LEFT_IDX==1-RIGHT_IDX &&
    abs(MOTOR_SIGN[0])==1 && abs(MOTOR_SIGN[1])==1 &&
    abs(ENC_SIGN[0])==1 && abs(ENC_SIGN[1])==1;
}
inline float setMotor(int i,float v){
  v=clipValue(v,-BATTERY_V,BATTERY_V);
  const float signedV=v*MOTOR_SIGN[i];
  digitalWrite(directionPin[i],signedV>0?HIGH:LOW);
  int duty=(int)(255.0f*fabs(signedV)/BATTERY_V+0.5f);
  duty=constrain(duty,0,255);
  analogWrite(pwmPin[i],duty);
  return (v>=0?1.0f:-1.0f)*duty*BATTERY_V/255.0f;
}
inline void stopMotors(){setMotor(0,0);setMotor(1,0);}
inline void initRobot(){
  pinMode(enablePin,OUTPUT); digitalWrite(enablePin,LOW);
  for(int i=0;i<2;i++){pinMode(directionPin[i],OUTPUT);pinMode(pwmPin[i],OUTPUT);}
  stopMotors();
  pinMode(encoderA1,INPUT_PULLUP);pinMode(encoderB1,INPUT_PULLUP);
  pinMode(encoderA2,INPUT_PULLUP);pinMode(encoderB2,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderA1),encoderISR1,CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderA2),encoderISR2,CHANGE);
  if(configValid())digitalWrite(enablePin,HIGH);
}
inline void readPose(float dt){
  long c[2];noInterrupts();c[0]=encoderCount[0];c[1]=encoderCount[1];interrupts();
  long r=ENC_SIGN[RIGHT_IDX]*c[RIGHT_IDX],l=ENC_SIGN[LEFT_IDX]*c[LEFT_IDX];
  const float scale=2.0f*PI/COUNTS_PER_REV;
  float dr=WHEEL_R*scale*(r-pose.cR), dl=WHEEL_R*scale*(l-pose.cL);
  float ds=(dr+dl)*0.5f, da=(dr-dl)/WHEEL_D;
  pose.x+=ds*cos(pose.phi+da*0.5f);pose.y+=ds*sin(pose.phi+da*0.5f);
  pose.rho=WHEEL_R*scale*(r+l)*0.5f;pose.phi=WHEEL_R*scale*(r-l)/WHEEL_D;
  pose.rhod=ds/dt;pose.phid=da/dt;pose.cR=r;pose.cL=l;
}
inline void printConfig(){
  Serial.print(F("# CONFIG r="));Serial.print(WHEEL_R,6);
  Serial.print(F(" d="));Serial.print(WHEEL_D,6);
  Serial.print(F(" cpr="));Serial.print(COUNTS_PER_REV,1);
  Serial.print(F(" batt="));Serial.print(BATTERY_V,3);
  Serial.print(F(" right="));Serial.print(RIGHT_IDX);
  Serial.print(F(" left="));Serial.print(LEFT_IDX);
  Serial.print(F(" motor0="));Serial.print(MOTOR_SIGN[0]);
  Serial.print(F(" motor1="));Serial.print(MOTOR_SIGN[1]);
  Serial.print(F(" enc0="));Serial.print(ENC_SIGN[0]);
  Serial.print(F(" enc1="));Serial.print(ENC_SIGN[1]);
  Serial.print(F(" identified="));Serial.println(MODEL_IDENTIFIED?1:0);
}
#endif
