// 6-DOF Robotic Arm - single-axis position/velocity low-pass filter test (AS5600 encoder)
// Author: Luca Obwegs

#include "AS5600.h"
#include "Wire.h"
#include <AccelStepper.h>

//axis 6
AccelStepper stepper6(1, A6, A7); 
#define EN_A6 A2
float pos6 = 0; float pos6deg = 0; float pos6deg_prev = 0; float lastPos6 = 0; float vel6deg = 0; float vel6deg_prev = 0; float zeroShift6 = 0; double i6 = -1; int MS6 = 16; 

AS5600 as5600;   //  use default Wire
int16_t value = 0;
unsigned long lastTime = 0;
int32_t deltaT = 0;
float a6_vel = 0.03;
float a6_pos = 0.03;

void selectEncoder(uint8_t bus){
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);

  pinMode(EN_A6, OUTPUT);
  digitalWrite(EN_A6, HIGH);
  stepper6.setMaxSpeed(MS6*i6*100); //steps/s
  stepper6.setAcceleration(MS6*i6*100); //steps/s^2


  //encoder setup
  Wire.begin();
  //axis 6
  selectEncoder(7);
  as5600.begin(4);
  as5600.setDirection(AS5600_CLOCK_WISE);

  calcZeroShift();
}

int pos_array[200] = {};
int vel_array[200] = {};
int ts = 100;
bool once = true;
int prev = 0;
int i = 0;

void loop() {
  calcPos();
  calcVelocity();
  Serial.print(pos6deg,1);
  Serial.print("\t");
  Serial.println(vel6deg,1);
}


void calcZeroShift() {
  for (int i = 0; i < 100; i++) {
    calcPos();
    calcVelocity();
    zeroShift6 = pos6;
  }
}

void calcPos()
{
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
  pos6deg = a6_pos*((pos6 - zeroShift6) * AS5600_RAW_TO_DEGREES / i6) + (1-a6_pos)*pos6deg_prev;
  lastPos6 = value;
  
}

void calcVelocity() {
  vel6deg = 1e6*a6_vel*(pos6deg - pos6deg_prev)/((micros() - lastTime)) + (1-a6_vel)*vel6deg_prev; //velocity in deg/s
  lastTime = micros();
  pos6deg_prev = pos6deg;
}



