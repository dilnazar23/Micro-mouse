#include <Arduino.h>
#include <VL6180X.h>
#include "pid_controller.hpp"
#include "Encoder.hpp"
#include "Motor.hpp"
#include "Tuple.hpp"

// Set up motors.
mtrn3100::Motor l_motor(2, 4, 3);
mtrn3100::Motor r_motor(7, 5, 6);

// Set up encoders.
void readLeftEncoder();
void readRightEncoder();
mtrn3100::Encoder l_encoder(18, 22, readLeftEncoder);
mtrn3100::Encoder r_encoder(19, 23, readRightEncoder);
void readLeftEncoder() { l_encoder.readEncoder(); }
void readRightEncoder() { r_encoder.readEncoder(); }
//
#define RANGE 1
const int address0=0x20;
const int address1=0x22;
int enablePin0 = 11;
int enablePin1 = 10;
VL6180X sensor0;
VL6180X sensor1;

void setup() {
     Serial.begin(9600);
    Serial3.begin(9600);
    //Serial.begin(9600);
    Wire.begin();
    l_motor.setPWM(0);
    r_motor.setPWM(0);
    delay(200);
    l_encoder.position = 0;
    l_encoder.position = 0;
    // Set up Lidar
    pinMode(enablePin0,OUTPUT);
    pinMode(enablePin1,OUTPUT);
    digitalWrite(enablePin0, LOW);
    digitalWrite(enablePin1, LOW); 
    delay(100); 
    // Sensor0
    Serial.println("Start Sensor 0");
    digitalWrite(enablePin0, HIGH);
    delay(50);
    sensor0.init();
    sensor0.configureDefault();
    sensor0.setAddress(address0);
    Serial.println(sensor0.readReg(0x212),HEX); // read I2C address
    sensor0.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
    sensor0.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
    sensor0.setTimeout(500);
    sensor0.stopContinuous();
    sensor0.setScaling(RANGE); // configure range or precision 1, 2 oder 3 mm
    delay(300);
    sensor0.startInterleavedContinuous(100);
    delay(100);

    // Sensor1
    Serial.println("Start Sensor 1");
    digitalWrite(enablePin1, HIGH);
    delay(50);
    sensor1.init();
    sensor1.configureDefault();
    sensor1.setAddress(address1);
    Serial.println(sensor1.readReg(0x212),HEX);
    sensor1.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
    sensor1.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
    sensor1.setTimeout(500);
    sensor1.stopContinuous();
    sensor1.setScaling(RANGE);
    delay(300);
    sensor1.startInterleavedContinuous(100);
    delay(100);
    Serial.println("Sensors ready! Start reading sensors in 3 seconds ...!");
    delay(500);

}

void loop() {
  // No pid
  ///// give the lowest speed, and turn until meet the condition
  // if reading decrease keep moving, stop when the reading is not decreasing
  while((sensor0.readRangeContinuousMillimeters()+sensor1.readRangeContinuousMillimeters()) < lidar_sum_prv){
    lidar_sum_prv = sensor0.readRangeContinuousMillimeters()+sensor1.readRangeContinuousMillimeters();
    l_motor.setPWM(30);
    r_motor.setPWM(30);
    // Should we keep this delay?
    delay(100);     
    // make the motor run , then read lidar
    
  };
  l_motor.setPWM(0);
  r_motor.setPWM(0);
  // another delay that I'm not sure to keep or not


}
