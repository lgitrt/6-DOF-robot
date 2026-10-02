// 6-DOF Robotic Arm - multi-axis stepper control with trapezoidal acceleration (FlexyStepper)
// Author: Luca Obwegs

#include <FlexyStepper.h>


//
// pin assignments
//
//Axis 1
byte directionPin1 = 45;
byte stepPin1 = 43;
double i1 = 2.35;
int MS1 = 16;
//Axis2
byte directionPin2 = 39;
byte stepPin2 = 41;
double i2 = 3.75;
int MS2 = 16;
//Axis3
byte directionPin3 = 47;
byte stepPin3 = 32;
double i3 = 5;
int MS3 = 16;
//Axis4
byte directionPin4 = A7;
byte stepPin4 = A6;
#define EN_A4 A2
double i4 = -2.8;
int MS4 = 16;
//Axis 5
byte directionPin5 = A1;
byte stepPin5 = A0;
double i5 = -2.1;
int MS5 = 16;
#define EN_A5 38
//Axis 6
byte directionPin6 = 48;
byte stepPin6 = 46;
double i6 = -1;
int MS6 = 16;
#define EN_A6 A8

//
// create the stepper motor object
//
FlexyStepper stepper1;
FlexyStepper stepper2;
FlexyStepper stepper3;
FlexyStepper stepper4;
FlexyStepper stepper5;
FlexyStepper stepper6;
float speedInStepsPerSecond_1;
long absSteps1;
float accelerationInStepsPerSecondPerSecond_1;
float speedInStepsPerSecond_2;
float accelerationInStepsPerSecondPerSecond_2;
long absSteps2;
float speedInStepsPerSecond_3;
long absSteps3;
float accelerationInStepsPerSecondPerSecond_3;
float speedInStepsPerSecond_4;
float accelerationInStepsPerSecondPerSecond_4;
long absSteps4;
float speedInStepsPerSecond_5;
long absSteps5;
float accelerationInStepsPerSecondPerSecond_5;
float speedInStepsPerSecond_6;
float accelerationInStepsPerSecondPerSecond_6;
long absSteps6;

//fans
#define FAN_PIN 9

long positions[6]; // Array of desired stepper positions
bool runSeq = false;
int del_0,del_1,del_2, del_3,del_4,del_5,del_6,del_7;
float p1, p2, p3, p4, p5, p6;
int absPos1 = 0;
int absPos2 = 0;
int absPos3 = 0;
int absPos4 = 0;
int absPos5 = 0;
int absPos6 = 0;

bool start = true;
bool newPos = false;

void setup() 
{
  Serial.begin(1000000);

  pinMode(EN_A4, OUTPUT);
  digitalWrite(EN_A4, LOW);
  pinMode(EN_A5, OUTPUT);
  digitalWrite(EN_A5, LOW);
  pinMode(EN_A6, OUTPUT);
  digitalWrite(EN_A6, LOW);


  stepper1.connectToPins(stepPin1, directionPin1);
  stepper1.setCurrentPositionInSteps(0);
  stepper1.setSpeedInStepsPerSecond(1000);
  stepper1.setAccelerationInStepsPerSecondPerSecond(300);
  stepper2.connectToPins(stepPin2, directionPin2);
  stepper2.setCurrentPositionInSteps(0);
  stepper2.setSpeedInStepsPerSecond(1000);
  stepper2.setAccelerationInStepsPerSecondPerSecond(300);
  stepper3.connectToPins(stepPin3, directionPin3);
  stepper3.setCurrentPositionInSteps(0);
  stepper3.setSpeedInStepsPerSecond(1000);
  stepper3.setAccelerationInStepsPerSecondPerSecond(300);
  stepper4.connectToPins(stepPin4, directionPin4);
  stepper4.setCurrentPositionInSteps(0);
  stepper4.setSpeedInStepsPerSecond(1000);
  stepper4.setAccelerationInStepsPerSecondPerSecond(300);
  stepper5.connectToPins(stepPin5, directionPin5);
  stepper5.setCurrentPositionInSteps(0);
  stepper5.setSpeedInStepsPerSecond(1000);
  stepper5.setAccelerationInStepsPerSecondPerSecond(300);
  stepper6.connectToPins(stepPin6, directionPin6);
  stepper6.setCurrentPositionInSteps(0);
  stepper6.setSpeedInStepsPerSecond(1000);
  stepper6.setAccelerationInStepsPerSecondPerSecond(300);

  //activate fan
  pinMode(FAN_PIN , OUTPUT);
  digitalWrite(FAN_PIN, HIGH);
  
}


void loop() 
{
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    del_0 = data.indexOf(",");
    del_1 = data.indexOf(",", del_0 + 1);
    del_2 = data.indexOf(",", del_1 + 1);
    del_3 = data.indexOf(",", del_2 + 1);
    del_4 = data.indexOf(",", del_3 + 1);
    del_5 = data.indexOf(",", del_4 + 1);
    del_6 = data.indexOf(",", del_5 + 1);
    del_7 = data.indexOf(",", del_6 + 1);

    //return initilization confirmation!
    if (data == "InitUart") {
      Serial.println("UartInitialized");
    }
    else if (data.substring(del_0 + 1, del_1) == "runSequence") {
        p1 = (data.substring(del_1 + 1, del_2)).toFloat();
        p2 = (data.substring(del_2 + 1, del_3)).toFloat();
        p3 = (data.substring(del_3 + 1, del_4)).toFloat();
        p4 = (data.substring(del_4 + 1, del_5)).toFloat();
        p5 = (data.substring(del_5 + 1, del_6)).toFloat();
        p6 = (data.substring(del_6 + 1, del_7)).toFloat();
        newPos = true;
    }
    else if (data.substring(del_0 + 1, del_1) == "runToPos") {
        p1 = (data.substring(del_1 + 1, del_2)).toFloat();
        p2 = (data.substring(del_2 + 1, del_3)).toFloat();
        p3 = (data.substring(del_3 + 1, del_4)).toFloat();
        p4 = (data.substring(del_4 + 1, del_5)).toFloat();
        p5 = (data.substring(del_5 + 1, del_6)).toFloat();
        p6 = (data.substring(del_6 + 1, del_7)).toFloat();
    }
    // calculate motor angles
    positions[0] = i1*MS1*(float(0.5555555555556))*p1;    
    positions[1] = i2*MS2*(float(0.5555555555556))*p2;    
    positions[2] = i3*MS3*(float(1.1111111111111))*p3;    
    positions[3] = i4*MS4*(float(0.5555555555556))*p4;
    positions[4] = -i5*MS5*(float(0.5555555555556))*p5;  
    positions[5] = i6*MS6*(float(0.5555555555556))*p6;
    moveXYWithCoordination(positions[0], positions[1], positions[2], positions[3], positions[4], positions[5], 4000, 800);
  }
  
  stepper1.processMovement();
  stepper2.processMovement();
  stepper3.processMovement();
  stepper4.processMovement();
  stepper5.processMovement();
  stepper6.processMovement();
  
  if (stepper1.motionComplete() && stepper2.motionComplete() && stepper3.motionComplete() && stepper4.motionComplete() && stepper5.motionComplete() && stepper6.motionComplete() && newPos) {
    Serial.println("posReached");
    newPos = false;
  }

  
}


void moveXYWithCoordination(long steps1, long steps2,long steps3, long steps4,long steps5, long steps6, float speedInStepsPerSecond, float accelerationInStepsPerSecondPerSecond)
{
  steps1 = steps1 - stepper1.getCurrentPositionInSteps();
  steps2 = steps2 - stepper2.getCurrentPositionInSteps();
  steps3 = steps3 - stepper3.getCurrentPositionInSteps();
  steps4 = steps4 - stepper4.getCurrentPositionInSteps();
  steps5 = steps5 - stepper5.getCurrentPositionInSteps();
  steps6 = steps6 - stepper6.getCurrentPositionInSteps();


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
  stepper1.setSpeedInStepsPerSecond(speedInStepsPerSecond_1);
  stepper1.setAccelerationInStepsPerSecondPerSecond(accelerationInStepsPerSecondPerSecond_1);
  stepper2.setSpeedInStepsPerSecond(speedInStepsPerSecond_2);
  stepper2.setAccelerationInStepsPerSecondPerSecond(accelerationInStepsPerSecondPerSecond_2);
  stepper3.setSpeedInStepsPerSecond(speedInStepsPerSecond_3);
  stepper3.setAccelerationInStepsPerSecondPerSecond(accelerationInStepsPerSecondPerSecond_3);
  stepper4.setSpeedInStepsPerSecond(speedInStepsPerSecond_4);
  stepper4.setAccelerationInStepsPerSecondPerSecond(accelerationInStepsPerSecondPerSecond_4);
  stepper5.setSpeedInStepsPerSecond(speedInStepsPerSecond_5);
  stepper5.setAccelerationInStepsPerSecondPerSecond(accelerationInStepsPerSecondPerSecond_5);
  stepper6.setSpeedInStepsPerSecond(speedInStepsPerSecond_6);
  stepper6.setAccelerationInStepsPerSecondPerSecond(accelerationInStepsPerSecondPerSecond_6);

  //set target position
  stepper1.setTargetPositionRelativeInSteps(steps1);
  stepper2.setTargetPositionRelativeInSteps(steps2);
  stepper3.setTargetPositionRelativeInSteps(steps3);
  stepper4.setTargetPositionRelativeInSteps(steps4);
  stepper5.setTargetPositionRelativeInSteps(steps5);
  stepper6.setTargetPositionRelativeInSteps(steps6);

}

