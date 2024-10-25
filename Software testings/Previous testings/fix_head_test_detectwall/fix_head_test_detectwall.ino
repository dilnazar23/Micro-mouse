#include <Wire.h>
#include <VL6180X.h>
#include "Motor.hpp"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "MAFUltrasonicSensor.hpp"
#include "UltrasonicSensor.hpp"
//DECLARE ROBOT GLOBAL VALUES
float x0,y0,h0;
// set up lidar
#define RANGE 1
#define address0 0x20
#define address1 0x22
int enablePin0 = 11;
int enablePin1 = 10;
VL6180X sensor0;
VL6180X sensor1;
//set up ultra
mtrn3100::UltrasonicSensor sensor(40, 38);
mtrn3100::MAFUltrasonicSensor<30> maf_sensor(sensor);

void detectWall() {
  float front;
  float left;
  float right;

  maf_sensor.sample();
  front = maf_sensor.value();
  if (maf_sensor.isReady()) {
   if (front < 150) {Serial.print("Front: ");Serial.println(front);}
  }
  left = sensor0.readRangeContinuousMillimeters();
  if (left <150) {Serial.print("Left: ");Serial.println(left);}
  right = sensor1.readRangeContinuousMillimeters();
  if (right < 150) {Serial.print("Right: ");Serial.println(right); }
  
    
  
  //else{return 0;}
}


void setup() {
   Serial.begin(9600);
   Serial3.begin(9600);
   Wire.begin();
    
    // Set up Lidar
    pinMode(enablePin0,OUTPUT);
    pinMode(enablePin1,OUTPUT);
    digitalWrite(enablePin0, LOW);
    digitalWrite(enablePin1, LOW); 
    delay(1000); 
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

}

void loop() {
  detectWall();

}
