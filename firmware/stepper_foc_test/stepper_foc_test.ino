// 6-DOF Robotic Arm - field-oriented control (FOC) feasibility test for a stepper joint
// Author: Luca Obwegs

#include <SimpleFOC.h>

void selectEncoder(uint8_t bus){
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}



//sensor
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// Stepper driver instance
StepperMotor motor = StepperMotor(8, 10);
StepperDriver4PWM driver = StepperDriver4PWM(29, 31, 33, 35, 7, 8);




void setup() {

  // initialize magnetic sensor hardware
  selectEncoder(7);
  sensor.init();
  selectEncoder(7);

  // pwm frequency to be used [Hz]
  driver.pwm_frequency = 20000;
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  // Max DC voltage allowed - default voltage_power_supply
  driver.voltage_limit = 12;
  
  // driver init
  driver.init();

  // init sensor
  // link the motor to the sensor
  /*motor.linkSensor(&sensor);

  // init driver
  // link the motor to the driver
  motor.linkDriver(&driver);
  
  // set control loop type to be used
  motor.controller = MotionControlType::velocity;
  // initialize motor
  motor.init();*/


  // align encoder and start FOC
  //motor.initFOC();
}

void loop() {
  // FOC algorithm function
  //motor.loopFOC();

  // velocity control loop function
  // setting the target velocity or 2rad/s
  //motor.move(2);
}
