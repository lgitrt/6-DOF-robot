//
//    FILE: axis6_encoder_test.ino
//  AUTHOR: Rob Tillaart (AS5600 library example), extended by Luca Obwegs
// PURPOSE: Read the AS5600 magnetic encoder on axis 6 through a TCA9548A
//          I2C multiplexer and report filtered position/velocity.
// PROJECT: 6-DOF Robotic Arm


#include "AS5600.h"
#include "Wire.h"
#include <AccelStepper.h>

AS5600 as5600;   //  use default Wire

// Per-axis gear ratio (i*) and microstepping (MS*) constants, and the running
// position/velocity state used by the low-pass filter in calcPosAndVel().
// Axes 1-5 are read-only here (no stepper attached); axis 6 is also driven as
// the test actuator below.
int32_t pos1 = 0; int32_t lastPos1 = 0; int32_t vel1 = 0; int32_t lastVel1 = 0; int32_t deltaA1 = 0; int32_t zeroShift1 = 0; double i1 = 2.35; int MS1 = 16;
int32_t pos2 = 0; int32_t lastPos2 = 0; int32_t vel2 = 0; int32_t lastVel2 = 0; int32_t deltaA2 = 0; int32_t zeroShift2 = 0; double i2 = 3.75; int MS2 = 16;
int32_t pos3 = 0; int32_t lastPos3 = 0; int32_t vel3 = 0; int32_t lastVel3 = 0; int32_t deltaA3 = 0; int32_t zeroShift3 = 0; double i3 = 5; int MS3 = 32;
int32_t pos4 = 0; int32_t lastPos4 = 0; int32_t vel4 = 0; int32_t lastVel4 = 0; int32_t deltaA4 = 0; int32_t zeroShift4 = 0; double i4 = -2.8; int MS4 = 16;
int32_t pos5 = 0; int32_t lastPos5 = 0; int32_t vel5 = 0; int32_t lastVel5 = 0; int32_t deltaA5 = 0; int32_t zeroShift5 = 0; double i5 = -2.1; int MS5 = 16;

// Axis 6 (test actuator): driven step/direction via AccelStepper.
AccelStepper stepper6(1, A6, A7);
#define EN_A6 A2
int32_t pos6 = 0; int32_t lastPos6 = 0; int32_t vel6 = 0; int32_t lastVel6 = 0; int32_t deltaA6 = 0; int32_t zeroShift6 = 0; double i6 = -1; int MS6 = 16;


int16_t value = 0;
int32_t timer = 0;
int32_t lastTime = 0;
int32_t deltaT = 0;

//lowpass filter
double a1 = 0.1; double a2 = 0.1; double a3 = 0.1; double a4 = 0.1; double a5 = 0.1; double a6 = 0.03;

void selectEncoder(uint8_t bus){
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}

void setup()
{
  Serial.begin(2000000);

  pinMode(EN_A6, OUTPUT);
  digitalWrite(EN_A6, LOW);
  stepper6.setMaxSpeed(MS6*i6*50); //steps/s
  stepper6.setAcceleration(MS6*i6*10); //steps/s^2
  stepper6.setCurrentPosition(0);


  Wire.begin();

  //set up encoders
  //axis 1
  selectEncoder(2);
  as5600.begin(4);
  as5600.setDirection(AS5600_CLOCK_WISE);
  //axis 2
  selectEncoder(3);
  as5600.begin(4);
  as5600.setDirection(AS5600_CLOCK_WISE);
  //axis 3
  selectEncoder(4);
  as5600.begin(4);
  as5600.setDirection(AS5600_CLOCK_WISE);
  //axis 4
  selectEncoder(5);
  as5600.begin(4);
  as5600.setDirection(AS5600_CLOCK_WISE);
  //axis 5
  selectEncoder(6);
  as5600.begin(4);
  as5600.setDirection(AS5600_CLOCK_WISE);
  //axis 6
  selectEncoder(7);
  as5600.begin(4);
  as5600.setDirection(AS5600_CLOCK_WISE);

  calcZeroShift();
  delay(5000);
}

int i = 0;

void loop()
{
  calcPosAndVel();

  // Report axis 6 encoder angle, commanded stepper angle, and the
  // tracking error between them (deg) over serial.
  Serial.print((pos6 - zeroShift6) * AS5600_RAW_TO_DEGREES / i6);
  Serial.print("\t");
  Serial.print(stepper6.currentPosition()*360/(200*MS6));
  Serial.print("\t");
  float error = ((pos6 - zeroShift6) * AS5600_RAW_TO_DEGREES / i6 - stepper6.currentPosition()*360/(200*MS6));
  Serial.println(error);

  if (i<=180) {
    stepper6.moveTo((i-error)*i6*MS6*(float(0.5555555555556)));
    stepper6.run();
  }
  if (stepper6.distanceToGo() == 0) {i++;}
  

}

void calcZeroShift() {
  for (int i = 0; i < 100; i++) {
    calcPosAndVel();
    zeroShift1 = pos1;
    zeroShift2 = pos2;
    zeroShift3 = pos3;
    zeroShift4 = pos4;
    zeroShift5 = pos5;
    zeroShift6 = pos6;
  }
}

void calcPosAndVel()
{
  //axis 1
  selectEncoder(2);
  value = as5600.rawAngle();
  if ((lastPos1 > 2048) && ( value < (lastPos1 - 2048)))
  {
    pos1 = pos1 + 4096 - lastPos1 + value;
  }
  else if ((value > 2048) && ( lastPos1 < (value - 2048)))
  {
    pos1 = pos1 - 4096 - lastPos1 + value;
  }
  else pos1 = pos1 - lastPos1 + value;

  deltaT = millis() - lastTime;
  deltaA1 =  pos1 - lastPos1;
  if (deltaA1 >  2048) deltaA1 -= 4096;
  if (deltaA1 < -2048) deltaA1 += 4096;
  vel1   = (deltaA1*1000) / deltaT;
  vel1 = a1*vel1 + (1-a1)*lastVel1; //lowpass filter
  if (abs(vel1 * AS5600_RAW_TO_DEGREES / i1)<0.5) {vel1=0;}
  lastVel1 = vel1;
  lastPos1 = value;
  
  //axis 2
  selectEncoder(3);
  value = as5600.rawAngle();
  if ((lastPos2 > 2048) && ( value < (lastPos2 - 2048)))
  {
    pos2 = pos2 + 4096 - lastPos2 + value;
  }
  else if ((value > 2048) && ( lastPos2 < (value - 2048)))
  {
    pos2 = pos2 - 4096 - lastPos2 + value;
  }
  else pos2 = pos2 - lastPos2 + value;
  deltaA2 =  pos2 - lastPos2;
  if (deltaA2 >  2048) deltaA2 -= 4096;
  if (deltaA2 < -2048) deltaA2 += 4096;
  vel2   = (deltaA2*1000) / deltaT;
  vel2 = a2*vel2 + (1-a2)*lastVel2; //lowpass filter
  if (abs(vel2 * AS5600_RAW_TO_DEGREES / i2)<0.5) {vel2=0;}
  lastVel2 = vel2;
  lastPos2 = value;
  
  //axis 3
  selectEncoder(4);
  value = as5600.rawAngle();
  if ((lastPos3 > 2048) && ( value < (lastPos3 - 2048)))
  {
    pos3 = pos3 + 4096 - lastPos3 + value;
  }
  else if ((value > 2048) && ( lastPos3 < (value - 2048)))
  {
    pos3 = pos3 - 4096 - lastPos3 + value;
  }
  else pos3 = pos3 - lastPos3 + value;
  deltaA3 =  pos3 - lastPos3;
  if (deltaA3 >  2048) deltaA3 -= 4096;
  if (deltaA3 < -2048) deltaA3 += 4096;
  vel3   = (deltaA3*1000) / deltaT;
  vel3 = a3*vel3 + (1-a3)*lastVel3; //lowpass filter
  if (abs(vel3 * AS5600_RAW_TO_DEGREES / i3)<0.5) {vel3=0;}
  lastVel3 = vel3;
  lastPos3 = value;

  //axis 4
  selectEncoder(5);
  value = as5600.rawAngle();
  if ((lastPos4 > 2048) && ( value < (lastPos4 - 2048)))
  {
    pos4 = pos4 + 4096 - lastPos4 + value;
  }
  else if ((value > 2048) && ( lastPos4 < (value - 2048)))
  {
    pos4 = pos4 - 4096 - lastPos4 + value;
  }
  else pos4 = pos4 - lastPos4 + value;
  deltaA4 =  pos4 - lastPos4;
  if (deltaA4 >  2048) deltaA4 -= 4096;
  if (deltaA4 < -2048) deltaA4 += 4096;
  vel4   = (deltaA4*1000) / deltaT;
  vel4 = a4*vel4 + (1-a4)*lastVel4; //lowpass filter
  if (abs(vel4 * AS5600_RAW_TO_DEGREES / i4)<0.5) {vel4=0;}
  lastVel4 = vel4;
  lastPos4 = value;

  //axis 5
  selectEncoder(6);
  value = as5600.rawAngle();
  if ((lastPos5 > 2048) && ( value < (lastPos5 - 2048)))
  {
    pos5 = pos5 + 4096 - lastPos5 + value;
  }
  else if ((value > 2048) && ( lastPos5 < (value - 2048)))
  {
    pos5 = pos5 - 4096 - lastPos5 + value;
  }
  else pos5 = pos5 - lastPos5 + value;
  deltaA5 =  pos5 - lastPos5;
  if (deltaA5 >  2048) deltaA5 -= 4096;
  if (deltaA5 < -2048) deltaA5 += 4096;
  vel5   = (deltaA5*1000) / deltaT;
  vel5 = a5*vel5 + (1-a5)*lastVel5; //lowpass filter
  if (abs(vel5 * AS5600_RAW_TO_DEGREES / i5)<0.5) {vel5=0;}
  lastVel5 = vel5;
  lastPos5 = value;

  //axis 6
  selectEncoder(7);
  value = as5600.rawAngle();
  if ((lastPos6 > 2048) && ( value < (lastPos6 - 2048)))
  {
    pos6 = pos6 + 4096 - lastPos6 + value;
  }
  else if ((value > 2048) && ( lastPos6 < (value - 2048)))
  {
    pos6 = pos6 - 4096 - lastPos6 + value;
  }
  else pos6 = pos6 - lastPos6 + value;
  deltaA6 =  pos6 - lastPos6;
  if (deltaA6 >  2048) deltaA6 -= 4096;
  if (deltaA6 < -2048) deltaA6 += 4096;
  vel6   = (deltaA6*1000) / deltaT;
  vel6 = a6*vel6 + (1-a6)*lastVel6; //lowpass filter
  if (abs(vel6 * AS5600_RAW_TO_DEGREES / i6)<0.5) {vel6=0;}
  lastVel6 = vel6;
  lastPos6 = value;

  lastTime = millis();
}



