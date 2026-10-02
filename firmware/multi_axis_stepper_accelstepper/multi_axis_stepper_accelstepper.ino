// 6-DOF Robotic Arm - multi-axis stepper control with AS5600 encoder feedback (AccelStepper)
// Author: Luca Obwegs

#include <AccelStepper.h>
#include "AS5600.h"
#include "Wire.h"

//encoder
AS5600 as5600;

//define stepper
//Axis1
AccelStepper stepper1(1, 43, 45);
#define EN_A1 16
double i1 = 2.35;
int MS1 = 16;
//Axis2
AccelStepper stepper2(1, 41, 39);
#define EN_A2 23
double i2 = 3.75;
int MS2 = 16;
//Axis3
AccelStepper stepper3(1, 32, 47);
#define EN_A3 17
double i3 = 5;
int MS3 = 16;
//Axis4
AccelStepper stepper4(1, A0, A1);
#define EN_A4 38
double i4 = -2.8;
int MS4 = 16;
//Axis5
AccelStepper stepper5(1, 46, 48);
#define EN_A5 A8
double i5 = -2.1;
int MS5 = 16;
//Axis6
AccelStepper stepper6(1, A6, A7); 
double i6 = -1;
int MS6 = 16;
#define EN_A6 A2

float pos1 = 0; float pos1deg = 0; float pos1deg_prev = 0; float lastPos1 = 0; float vel1deg = 0; float vel1deg_prev = 0; float encoderShift1 = 0; 
float pos2 = 0; float pos2deg = 0; float pos2deg_prev = 0; float lastPos2 = 0; float vel2deg = 0; float vel2deg_prev = 0; float encoderShift2 = 0; 
float pos3 = 0; float pos3deg = 0; float pos3deg_prev = 0; float lastPos3 = 0; float vel3deg = 0; float vel3deg_prev = 0; float encoderShift3 = 0; 
float pos4 = 0; float pos4deg = 0; float pos4deg_prev = 0; float lastPos4 = 0; float vel4deg = 0; float vel4deg_prev = 0; float encoderShift4 = 0; 
float pos5 = 0; float pos5deg = 0; float pos5deg_prev = 0; float lastPos5 = 0; float vel5deg = 0; float vel5deg_prev = 0; float encoderShift5 = 0; 
float pos6 = 0; float pos6deg = 0; float pos6deg_prev = 0; float lastPos6 = 0; float vel6deg = 0; float vel6deg_prev = 0; float encoderShift6 = 0; 

//lowpass filter
float a1_vel = 0.02; float a1_pos = 0.02;
float a2_vel = 0.02; float a2_pos = 0.02;
float a3_vel = 0.02; float a3_pos = 0.02;
float a4_vel = 0.02; float a4_pos = 0.02;
float a5_vel = 0.02; float a5_pos = 0.02;
float a6_vel = 0.02; float a6_pos = 0.02;

//fans
#define FAN_PIN 9

float speedInStepsPerSecond_1; long absSteps1; float accelerationInStepsPerSecondPerSecond_1;
float speedInStepsPerSecond_2; float accelerationInStepsPerSecondPerSecond_2; long absSteps2;
float speedInStepsPerSecond_3; long absSteps3; float accelerationInStepsPerSecondPerSecond_3;
float speedInStepsPerSecond_4; float accelerationInStepsPerSecondPerSecond_4; long absSteps4;
float speedInStepsPerSecond_5; long absSteps5; float accelerationInStepsPerSecondPerSecond_5;
float speedInStepsPerSecond_6; float accelerationInStepsPerSecondPerSecond_6; long absSteps6;

long positions[6]; // Array of desired stepper positions
bool runSeq = false;
uint16_t del_0,del_1,del_2, del_3,del_4,del_5,del_6,del_7,del_8,del_9,del_10,del_11,del_12,del_13;
float p1, p2, p3, p4, p5, p6, vel, acc;
int8_t absPos1 = 0; int8_t absPos2 = 0; int8_t absPos3 = 0; int8_t absPos4 = 0; int8_t absPos5 = 0; int8_t absPos6 = 0;
bool newPos = false; bool record = false;
int16_t value = 0; unsigned long prevMeasurement = 0;


void selectEncoder(uint8_t bus){
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}


void setup() 
{
  Serial.begin(1000000);

  //set stepper enable pins
  pinMode(EN_A1, OUTPUT); pinMode(EN_A2, OUTPUT); pinMode(EN_A3, OUTPUT); pinMode(EN_A4, OUTPUT); pinMode(EN_A5, OUTPUT); pinMode(EN_A6, OUTPUT);

  //set max values
  stepper1.setMaxSpeed(MS1*i1*10); stepper1.setAcceleration(MS1*i1*10);
  stepper2.setMaxSpeed(MS2*i2*10); stepper2.setAcceleration(MS2*i2*10);
  stepper3.setMaxSpeed(MS3*i3*10); stepper3.setAcceleration(MS3*i3*10);
  stepper4.setMaxSpeed(MS4*i4*10); stepper4.setAcceleration(MS4*i4*10);
  stepper5.setMaxSpeed(MS5*i5*10); stepper5.setAcceleration(MS5*i5*10);
  stepper6.setMaxSpeed(MS6*i6*10); stepper6.setAcceleration(MS6*i6*10);

  //activate fan
  pinMode(FAN_PIN , OUTPUT); digitalWrite(FAN_PIN, HIGH);

  //encoder setup
  Wire.begin();
  //axis 1
  selectEncoder(2); as5600.begin(4); as5600.setDirection(AS5600_CLOCK_WISE);
  //axis 2
  selectEncoder(3); as5600.begin(4); as5600.setDirection(AS5600_CLOCK_WISE);
  //axis 3
  selectEncoder(4); as5600.begin(4); as5600.setDirection(AS5600_CLOCK_WISE);
  //axis 4
  selectEncoder(5); as5600.begin(4); as5600.setDirection(AS5600_CLOCK_WISE);
  //axis 5
  selectEncoder(6); as5600.begin(4); as5600.setDirection(AS5600_CLOCK_WISE);
  //axis 6
  selectEncoder(7); as5600.begin(4); as5600.setDirection(AS5600_CLOCK_WISE);

  delay(1000);

  //diable motors
  disable_motors();

  enable_motors();

  //set encoder zero position
  calcEncoderShift();
}


void loop() 
{
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    del_0 = data.indexOf(","); del_1 = data.indexOf(",", del_0 + 1); del_2 = data.indexOf(",", del_1 + 1);
    del_3 = data.indexOf(",", del_2 + 1); del_4 = data.indexOf(",", del_3 + 1); del_5 = data.indexOf(",", del_4 + 1);
    del_6 = data.indexOf(",", del_5 + 1); del_7 = data.indexOf(",", del_6 + 1); del_8 = data.indexOf(",", del_7 + 1);
    del_9 = data.indexOf(",", del_8 + 1); del_10 = data.indexOf(",", del_9 + 1); del_11 = data.indexOf(",", del_10 + 1);
    del_12 = data.indexOf(",", del_11 + 1); del_13 = data.indexOf(",", del_12 + 1);

    //return initilization confirmation!
    if (data == "InitUart") {Serial.println("UartInitialized");}
    else if (data.substring(del_0 + 1, del_1) == "runToPos") {
        //extract pos, vel, acc form String
        p1 = (data.substring(del_1 + 1, del_2)).toFloat();
        p2 = (data.substring(del_2 + 1, del_3)).toFloat();
        p3 = (data.substring(del_3 + 1, del_4)).toFloat();
        p4 = (data.substring(del_4 + 1, del_5)).toFloat();
        p5 = (data.substring(del_5 + 1, del_6)).toFloat();
        p6 = (data.substring(del_6 + 1, del_7)).toFloat();
        vel = (data.substring(del_7 + 1, del_8)).toFloat() * 16 * float(0.5555555555556);
        acc = (data.substring(del_8 + 1, del_9)).toFloat() * 16 * float(0.5555555555556);
        // calculate motor angles
        positions[0] = i1*MS1*(float(0.5555555555556))*p1;    
        positions[1] = -i2*MS2*(float(0.5555555555556))*p2;    
        positions[2] = i3*MS3*(float(0.5555555555556))*p3;    
        positions[3] = i4*MS4*(float(0.5555555555556))*p4;
        positions[4] = -i5*MS5*(float(0.5555555555556))*p5;  
        positions[5] = i6*MS6*(float(0.5555555555556))*p6;
        //calculate acc and vel of each motor
        moveXYWithCoordination(positions[0], positions[1], positions[2], positions[3], positions[4], positions[5], vel, acc);
    }
    else if (data == "disableMotors") {disable_motors();}
    else if (data == "enableMotors") {enable_motors();}
    else if (data == "startRecord") {record = true;}
    else if (data == "stopRecord") {record = false;}
    else if (data == "setHome" && pos_reached()) {
      stepper1.setCurrentPosition(0);stepper2.setCurrentPosition(0);stepper3.setCurrentPosition(0);stepper4.setCurrentPosition(0);stepper5.setCurrentPosition(0);stepper6.setCurrentPosition(0);
      Serial.println("homeSet");
    }
  }
  
  //execute controller
  stepper1.run(); stepper2.run(); stepper3.run(); stepper4.run(); stepper5.run(); stepper6.run();

  if (record) {
    //get encoder values
    calcPosition(); calcVelocity();
    //publish current encoder values
    Serial.println(String(pos1deg) + "," + String(pos2deg) + "," + String(pos3deg) + "," + String(pos4deg) + "," + String(pos5deg) + ","
                 + String(pos6deg) + "," + String(vel1deg) + "," + String(vel2deg) + "," + String(vel3deg) + "," + String(vel4deg) + "," 
                 + String(vel5deg) + "," + String(vel6deg));
  }

  

}

void moveXYWithCoordination(long steps1, long steps2,long steps3, long steps4,long steps5, long steps6, float speedInStepsPerSecond, float accelerationInStepsPerSecondPerSecond) {
  steps1 = steps1 - stepper1.currentPosition();
  steps2 = steps2 - stepper2.currentPosition();
  steps3 = steps3 - stepper3.currentPosition();
  steps4 = steps4 - stepper4.currentPosition();
  steps5 = steps5 - stepper5.currentPosition();
  steps6 = steps6 - stepper6.currentPosition();

  //
  // setup initial speed and acceleration values
  //
  speedInStepsPerSecond_1 = speedInStepsPerSecond;
  accelerationInStepsPerSecondPerSecond_1 = accelerationInStepsPerSecondPerSecond;
  speedInStepsPerSecond_2 = speedInStepsPerSecond;
  accelerationInStepsPerSecondPerSecond_2 = accelerationInStepsPerSecondPerSecond;
  speedInStepsPerSecond_3 = speedInStepsPerSecond;
  accelerationInStepsPerSecondPerSecond_3 = accelerationInStepsPerSecondPerSecond;
  speedInStepsPerSecond_4 = speedInStepsPerSecond;
  accelerationInStepsPerSecondPerSecond_4 = accelerationInStepsPerSecondPerSecond;
  speedInStepsPerSecond_5 = speedInStepsPerSecond;
  accelerationInStepsPerSecondPerSecond_5 = accelerationInStepsPerSecondPerSecond;
  speedInStepsPerSecond_6 = speedInStepsPerSecond;
  accelerationInStepsPerSecondPerSecond_6 = accelerationInStepsPerSecondPerSecond;

  //
  // determine how many steps each motor is moving
  //
  if (steps1 >= 0) {absSteps1 = steps1;} else {absSteps1 = -steps1;}
  if (steps2 >= 0) {absSteps2 = steps2;} else {absSteps2 = -steps2;}
  if (steps3 >= 0) {absSteps3 = steps3;} else {absSteps3 = -steps3;}
  if (steps4 >= 0) {absSteps4 = steps4;} else {absSteps4 = -steps4;}
  if (steps5 >= 0) {absSteps5 = steps5;} else {absSteps5 = -steps5;}
  if (steps6 >= 0) {absSteps6 = steps6;} else {absSteps6 = -steps6;}

  //
  // determine which motor is traveling the farthest, then slow down the
  // speed rates for the motor moving the shortest distance and slow down the other motors
  //
  if ((absSteps1 > absSteps2) && (absSteps1 > absSteps3) && (absSteps1 > absSteps4) && (absSteps1 > absSteps5) && (absSteps1 > absSteps6) && (steps1 != 0))
  {
    float scaler2 = (float) absSteps2 / (float) absSteps1;
    float scaler3 = (float) absSteps3 / (float) absSteps1;
    float scaler4 = (float) absSteps4 / (float) absSteps1;
    float scaler5 = (float) absSteps5 / (float) absSteps1;
    float scaler6 = (float) absSteps6 / (float) absSteps1;
    speedInStepsPerSecond_2 = speedInStepsPerSecond_2 * scaler2;
    accelerationInStepsPerSecondPerSecond_2 = accelerationInStepsPerSecondPerSecond_2 * scaler2;
    speedInStepsPerSecond_3 = speedInStepsPerSecond_3 * scaler3;
    accelerationInStepsPerSecondPerSecond_3 = accelerationInStepsPerSecondPerSecond_3 * scaler3;
    speedInStepsPerSecond_4 = speedInStepsPerSecond_4 * scaler4;
    accelerationInStepsPerSecondPerSecond_4 = accelerationInStepsPerSecondPerSecond_4 * scaler4;
    speedInStepsPerSecond_5 = speedInStepsPerSecond_5 * scaler5;
    accelerationInStepsPerSecondPerSecond_5 = accelerationInStepsPerSecondPerSecond_5 * scaler5;
    speedInStepsPerSecond_6 = speedInStepsPerSecond_6 * scaler6;
    accelerationInStepsPerSecondPerSecond_6 = accelerationInStepsPerSecondPerSecond_6 * scaler6;
  }
  else if ((absSteps2 > absSteps1) && (absSteps2 > absSteps3) && (absSteps2 > absSteps4) && (absSteps2 > absSteps5) && (absSteps2 > absSteps6) && (steps2 != 0))
  {
    float scaler1 = (float) absSteps1 / (float) absSteps2;
    float scaler3 = (float) absSteps3 / (float) absSteps2;
    float scaler4 = (float) absSteps4 / (float) absSteps2;
    float scaler5 = (float) absSteps5 / (float) absSteps2;
    float scaler6 = (float) absSteps6 / (float) absSteps2;
    speedInStepsPerSecond_1 = speedInStepsPerSecond_1 * scaler1;
    accelerationInStepsPerSecondPerSecond_1 = accelerationInStepsPerSecondPerSecond_1 * scaler1;
    speedInStepsPerSecond_3 = speedInStepsPerSecond_3 * scaler3;
    accelerationInStepsPerSecondPerSecond_3 = accelerationInStepsPerSecondPerSecond_3 * scaler3;
    speedInStepsPerSecond_4 = speedInStepsPerSecond_4 * scaler4;
    accelerationInStepsPerSecondPerSecond_4 = accelerationInStepsPerSecondPerSecond_4 * scaler4;
    speedInStepsPerSecond_5 = speedInStepsPerSecond_5 * scaler5;
    accelerationInStepsPerSecondPerSecond_5 = accelerationInStepsPerSecondPerSecond_5 * scaler5;
    speedInStepsPerSecond_6 = speedInStepsPerSecond_6 * scaler6;
    accelerationInStepsPerSecondPerSecond_6 = accelerationInStepsPerSecondPerSecond_6 * scaler6;
  }
  else if ((absSteps3 > absSteps1) && (absSteps3 > absSteps2) && (absSteps3 > absSteps4) && (absSteps3 > absSteps5) && (absSteps3 > absSteps6) && (steps3 != 0))
  {
    float scaler1 = (float) absSteps1 / (float) absSteps3;
    float scaler2 = (float) absSteps2 / (float) absSteps3;
    float scaler4 = (float) absSteps4 / (float) absSteps3;
    float scaler5 = (float) absSteps5 / (float) absSteps3;
    float scaler6 = (float) absSteps6 / (float) absSteps3;
    speedInStepsPerSecond_1 = speedInStepsPerSecond_1 * scaler1;
    accelerationInStepsPerSecondPerSecond_1 = accelerationInStepsPerSecondPerSecond_1 * scaler1;
    speedInStepsPerSecond_2 = speedInStepsPerSecond_2 * scaler2;
    accelerationInStepsPerSecondPerSecond_2 = accelerationInStepsPerSecondPerSecond_2 * scaler2;
    speedInStepsPerSecond_4 = speedInStepsPerSecond_4 * scaler4;
    accelerationInStepsPerSecondPerSecond_4 = accelerationInStepsPerSecondPerSecond_4 * scaler4;
    speedInStepsPerSecond_5 = speedInStepsPerSecond_5 * scaler5;
    accelerationInStepsPerSecondPerSecond_5 = accelerationInStepsPerSecondPerSecond_5 * scaler5;
    speedInStepsPerSecond_6 = speedInStepsPerSecond_6 * scaler6;
    accelerationInStepsPerSecondPerSecond_6 = accelerationInStepsPerSecondPerSecond_6 * scaler6;
  }
  else if ((absSteps4 > absSteps1) && (absSteps4 > absSteps2) && (absSteps4 > absSteps3) && (absSteps4 > absSteps5) && (absSteps4 > absSteps6) && (steps4 != 0))
  {
    float scaler1 = (float) absSteps1 / (float) absSteps4;
    float scaler2 = (float) absSteps2 / (float) absSteps4;
    float scaler3 = (float) absSteps3 / (float) absSteps4;
    float scaler5 = (float) absSteps5 / (float) absSteps4;
    float scaler6 = (float) absSteps6 / (float) absSteps4;
    speedInStepsPerSecond_1 = speedInStepsPerSecond_1 * scaler1;
    accelerationInStepsPerSecondPerSecond_1 = accelerationInStepsPerSecondPerSecond_1 * scaler1;
    speedInStepsPerSecond_2 = speedInStepsPerSecond_2 * scaler2;
    accelerationInStepsPerSecondPerSecond_2 = accelerationInStepsPerSecondPerSecond_2 * scaler2;
    speedInStepsPerSecond_3 = speedInStepsPerSecond_3 * scaler3;
    accelerationInStepsPerSecondPerSecond_3 = accelerationInStepsPerSecondPerSecond_3 * scaler3;
    speedInStepsPerSecond_5 = speedInStepsPerSecond_5 * scaler5;
    accelerationInStepsPerSecondPerSecond_5 = accelerationInStepsPerSecondPerSecond_5 * scaler5;
    speedInStepsPerSecond_6 = speedInStepsPerSecond_6 * scaler6;
    accelerationInStepsPerSecondPerSecond_6 = accelerationInStepsPerSecondPerSecond_6 * scaler6;
  }
  else if ((absSteps5 > absSteps1) && (absSteps5 > absSteps2) && (absSteps5 > absSteps3) && (absSteps5 > absSteps4) && (absSteps4 > absSteps6) && (steps5 != 0))
  {
    float scaler1 = (float) absSteps1 / (float) absSteps5;
    float scaler2 = (float) absSteps2 / (float) absSteps5;
    float scaler3 = (float) absSteps3 / (float) absSteps5;
    float scaler4 = (float) absSteps4 / (float) absSteps5;
    float scaler6 = (float) absSteps6 / (float) absSteps5;
    speedInStepsPerSecond_1 = speedInStepsPerSecond_1 * scaler1;
    accelerationInStepsPerSecondPerSecond_1 = accelerationInStepsPerSecondPerSecond_1 * scaler1;
    speedInStepsPerSecond_2 = speedInStepsPerSecond_2 * scaler2;
    accelerationInStepsPerSecondPerSecond_2 = accelerationInStepsPerSecondPerSecond_2 * scaler2;
    speedInStepsPerSecond_3 = speedInStepsPerSecond_3 * scaler3;
    accelerationInStepsPerSecondPerSecond_3 = accelerationInStepsPerSecondPerSecond_3 * scaler3;
    speedInStepsPerSecond_4 = speedInStepsPerSecond_4 * scaler4;
    accelerationInStepsPerSecondPerSecond_4 = accelerationInStepsPerSecondPerSecond_4 * scaler4;
    speedInStepsPerSecond_6 = speedInStepsPerSecond_6 * scaler6;
    accelerationInStepsPerSecondPerSecond_6 = accelerationInStepsPerSecondPerSecond_6 * scaler6;
  }
  else if ((absSteps6 > absSteps1) && (absSteps6 > absSteps2) && (absSteps6 > absSteps3) && (absSteps6 > absSteps4) && (absSteps6 > absSteps5) && (steps6 != 0))
  {
    float scaler1 = (float) absSteps1 / (float) absSteps6;
    float scaler2 = (float) absSteps2 / (float) absSteps6;
    float scaler3 = (float) absSteps3 / (float) absSteps6;
    float scaler4 = (float) absSteps4 / (float) absSteps6;
    float scaler5 = (float) absSteps5 / (float) absSteps6;
    speedInStepsPerSecond_1 = speedInStepsPerSecond_1 * scaler1;
    accelerationInStepsPerSecondPerSecond_1 = accelerationInStepsPerSecondPerSecond_1 * scaler1;
    speedInStepsPerSecond_2 = speedInStepsPerSecond_2 * scaler2;
    accelerationInStepsPerSecondPerSecond_2 = accelerationInStepsPerSecondPerSecond_2 * scaler2;
    speedInStepsPerSecond_3 = speedInStepsPerSecond_3 * scaler3;
    accelerationInStepsPerSecondPerSecond_3 = accelerationInStepsPerSecondPerSecond_3 * scaler3;
    speedInStepsPerSecond_4 = speedInStepsPerSecond_4 * scaler4;
    accelerationInStepsPerSecondPerSecond_4 = accelerationInStepsPerSecondPerSecond_4 * scaler4;
    speedInStepsPerSecond_5 = speedInStepsPerSecond_5 * scaler5;
    accelerationInStepsPerSecondPerSecond_5 = accelerationInStepsPerSecondPerSecond_5 * scaler5;
  }
  
  //set speeds and accelerations
  stepper1.setMaxSpeed(speedInStepsPerSecond_1);
  stepper1.setAcceleration(accelerationInStepsPerSecondPerSecond_1);
  stepper2.setMaxSpeed(speedInStepsPerSecond_2);
  stepper2.setAcceleration(accelerationInStepsPerSecondPerSecond_2);
  stepper3.setMaxSpeed(speedInStepsPerSecond_3*1.2);
  stepper3.setAcceleration(accelerationInStepsPerSecondPerSecond_3*1.2);
  stepper4.setMaxSpeed(speedInStepsPerSecond_4);
  stepper4.setAcceleration(accelerationInStepsPerSecondPerSecond_4);
  stepper5.setMaxSpeed(speedInStepsPerSecond_5);
  stepper5.setAcceleration(accelerationInStepsPerSecondPerSecond_5);
  stepper6.setMaxSpeed(speedInStepsPerSecond_6);
  stepper6.setAcceleration(accelerationInStepsPerSecondPerSecond_6);

  //set target position
  stepper1.move(steps1);
  stepper2.move(steps2);
  stepper3.move(steps3);
  stepper4.move(steps4);
  stepper5.move(steps5);
  stepper6.move(steps6);

}

void enable_motors() {
  digitalWrite(EN_A1, LOW);digitalWrite(EN_A2, LOW);digitalWrite(EN_A3, HIGH);digitalWrite(EN_A4, LOW);digitalWrite(EN_A5, LOW);digitalWrite(EN_A6, LOW);
  //digitalWrite(EN_A1, HIGH);digitalWrite(EN_A2, HIGH);digitalWrite(EN_A3, LOW);digitalWrite(EN_A4, HIGH);digitalWrite(EN_A5, HIGH);digitalWrite(EN_A6, LOW);
}

void disable_motors() {
  digitalWrite(EN_A1, HIGH);digitalWrite(EN_A2, HIGH);digitalWrite(EN_A3, LOW);digitalWrite(EN_A4, HIGH);digitalWrite(EN_A5, HIGH);digitalWrite(EN_A6, HIGH);
}

bool pos_reached() {
  if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0 && stepper3.distanceToGo() == 0 && stepper4.distanceToGo() == 0 && stepper5.distanceToGo() == 0 && stepper6.distanceToGo() == 0) {
    return true;
  }
  return false;
}

void calcPosition() {
  //axis 1
  selectEncoder(2);
  value = as5600.rawAngle();
  if ((lastPos1 > 2048) && ( value < (lastPos1 - 2048)))
  {
    pos1 = pos1 + 4091 - lastPos1 + value;
  }
  else if ((value > 2048) && ( lastPos1 < (value - 2048)))
  {
    pos1 = pos1 - 4091 - lastPos1 + value;
  }
  else pos1 = pos1 - lastPos1 + value;
  pos1deg = a1_pos*((pos1 - encoderShift1) * AS5600_RAW_TO_DEGREES / i1) + (1-a1_pos)*pos1deg_prev;
  lastPos1 = value;

  //axis 2  
  selectEncoder(3);
  value = as5600.rawAngle();
  if ((lastPos2 > 2048) && ( value < (lastPos2 - 2048)))
  {
    pos2 = pos2 + 4092 - lastPos2 + value;
  }
  else if ((value > 2048) && ( lastPos2 < (value - 2048)))
  {
    pos2 = pos2 - 4092 - lastPos2 + value;
  }
  else pos2 = pos2 - lastPos2 + value;
  pos2deg = a2_pos*((pos2 - encoderShift2) * AS5600_RAW_TO_DEGREES / i2) + (1-a2_pos)*pos2deg_prev;
  lastPos2 = value;

  //axis 3
  selectEncoder(4);
  value = as5600.rawAngle();
  if ((lastPos3 > 2048) && ( value < (lastPos3 - 2048)))
  {
    pos3 = pos3 + 4093 - lastPos3 + value;
  }
  else if ((value > 2048) && ( lastPos3 < (value - 2048)))
  {
    pos3 = pos3 - 4093 - lastPos3 + value;
  }
  else pos3 = pos3 - lastPos3 + value;
  pos3deg = a3_pos*((pos3 - encoderShift3) * AS5600_RAW_TO_DEGREES / i3) + (1-a3_pos)*pos3deg_prev;
  lastPos3 = value;

  //axis 4
  selectEncoder(5);
  value = as5600.rawAngle();
  if ((lastPos4 > 2048) && ( value < (lastPos4 - 2048)))
  {
    pos4 = pos4 + 4094 - lastPos4 + value;
  }
  else if ((value > 2048) && ( lastPos4 < (value - 2048)))
  {
    pos4 = pos4 - 4094 - lastPos4 + value;
  }
  else pos4 = pos4 - lastPos4 + value;
  pos4deg = a4_pos*((pos4 - encoderShift4) * AS5600_RAW_TO_DEGREES / i4) + (1-a4_pos)*pos4deg_prev;
  lastPos4 = value;

  //axis 5
  selectEncoder(6);
  value = as5600.rawAngle();
  if ((lastPos5 > 2048) && ( value < (lastPos5 - 2048)))
  {
    pos5 = pos5 + 4095 - lastPos5 + value;
  }
  else if ((value > 2048) && ( lastPos5 < (value - 2048)))
  {
    pos5 = pos5 - 4095 - lastPos5 + value;
  }
  else pos5 = pos5 - lastPos5 + value;
  pos5deg = a5_pos*((pos5 - encoderShift5) * AS5600_RAW_TO_DEGREES / i5) + (1-a5_pos)*pos5deg_prev;
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
  pos6deg = a6_pos*((pos6 - encoderShift6) * AS5600_RAW_TO_DEGREES / i6) + (1-a6_pos)*pos6deg_prev;
  lastPos6 = value;
  
}

void calcVelocity() {
  vel1deg = 1e6*a1_vel*(pos1deg - pos1deg_prev)/((micros() - prevMeasurement)) + (1-a1_vel)*vel1deg_prev; //velocity in deg/s
  vel2deg = 1e6*a2_vel*(pos2deg - pos2deg_prev)/((micros() - prevMeasurement)) + (1-a2_vel)*vel2deg_prev; //velocity in deg/s
  vel3deg = 1e6*a3_vel*(pos3deg - pos3deg_prev)/((micros() - prevMeasurement)) + (1-a3_vel)*vel3deg_prev; //velocity in deg/s
  vel4deg = 1e6*a4_vel*(pos4deg - pos4deg_prev)/((micros() - prevMeasurement)) + (1-a4_vel)*vel4deg_prev; //velocity in deg/s
  vel5deg = 1e6*a5_vel*(pos5deg - pos5deg_prev)/((micros() - prevMeasurement)) + (1-a5_vel)*vel5deg_prev; //velocity in deg/s
  vel6deg = 1e6*a6_vel*(pos6deg - pos6deg_prev)/((micros() - prevMeasurement)) + (1-a6_vel)*vel6deg_prev; //velocity in deg/s

  prevMeasurement = micros();
  pos1deg_prev = pos1deg;
  pos2deg_prev = pos2deg;
  pos3deg_prev = pos3deg;
  pos4deg_prev = pos4deg;
  pos5deg_prev = pos5deg;
  pos6deg_prev = pos6deg;
}

void calcEncoderShift() {
  for (int i = 0; i < 100; i++) {
    calcPosition();
    calcVelocity();
    encoderShift1 = pos1;
    encoderShift2 = pos2;
    encoderShift3 = pos3;
    encoderShift4 = pos4;
    encoderShift5 = pos5;
    encoderShift6 = pos6;
  }
}


