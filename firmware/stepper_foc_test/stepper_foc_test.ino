// 6-DOF Robotic Arm - field-oriented control (FOC) feasibility test for a stepper joint
// Author: Luca Obwegs

#include <SimpleFOC.h>

void selectEncoder(uint8_t bus){
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
}

// Magnetic position sensor
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// Stepper driver instance
StepperMotor motor = StepperMotor(8, 10);
StepperDriver4PWM driver = StepperDriver4PWM(29, 31, 33, 35, 7, 8);

void setup() {
  // Initialize the magnetic position sensor on the TCA9548A mux channel used by this joint.
  selectEncoder(7);
  sensor.init();
  selectEncoder(7);

  // PWM frequency to be used [Hz]
  driver.pwm_frequency = 20000;
  // Power supply voltage [V]
  driver.voltage_power_supply = 12;
  // Max DC voltage allowed - default voltage_power_supply
  driver.voltage_limit = 12;

  driver.init();

  // NOTE: this sketch only verified the sensor/driver initialization above.
  // Closing the FOC velocity loop (motor.linkSensor/linkDriver/init/initFOC
  // below, and motor.loopFOC()/move() in loop()) was not completed - FOC
  // control of a stepper joint was explored as a feasibility study and not
  // adopted for the final design, so this code is left commented out as a
  // starting point for anyone revisiting the idea.

  // motor.linkSensor(&sensor);
  // motor.linkDriver(&driver);
  // motor.controller = MotionControlType::velocity;
  // motor.init();
  // motor.initFOC();
}

void loop() {
  // motor.loopFOC();
  // motor.move(2); // target velocity in rad/s
}
