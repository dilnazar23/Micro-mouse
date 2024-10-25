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
//set up imu
MPU6050 imu;
//set up ultra
mtrn3100::UltrasonicSensor sensor(40, 38);
mtrn3100::MAFUltrasonicSensor<60> maf_sensor(sensor);
//set up motor
mtrn3100::Motor r_motor(2, 4, 3);
mtrn3100::Motor l_motor(7, 5, 6);

void fixHead(){
  char wall = detectWall(); 
  switch (wall){
    case 'L':
      fixHead_lidar(true);
      break;
    case 'R':
      fixHead_lidar(false);
      break;
    case 'F':
      fixHead_ultra();
      break;
    case '0':
      fixHead_IMU();
      break;
  }
}

char detectWall() {
  float front;
  float left;
  float right;

  maf_sensor.sample();
  float filtered_dis = maf_sensor.value();
  if (maf_sensor.isReady()) {
    front = filtered_dis;
  }
  left = sensor0.readRangeContinuousMillimeters();
  right = sensor1.readRangeContinuousMillimeters();
  
  if (left <150) {return 'L';}
  if (right < 150) {return 'R';  }  
  if (front < 150) {return 'F';}
  else{return 0;}
}

bool is_turnLeft(){     // use imu to tell me where should i turn to fix head
  int16_t h_imu=imu.getRotationZ();//read imu
  bool result;
  h_imu<h0 ?  result=true: result=false ;   
  return result;
}
// FIX HEADING WITH LIDAR
//--- READ IMU
//--- CHECK TURN LEFT OR STOP
//--- TURN UNTILL LIDAR READING GO UP
//--- REPEAT TILL LIDAR READING GOOD
float readLIDAR_sum(){
  return sensor0.readRangeContinuousMillimeters()+sensor1.readRangeContinuousMillimeters();
}
void fix_turnLeft(){
  l_motor.setPWM(-30);
  r_motor.setPWM(30);
}
void fix_turnRight(){
  l_motor.setPWM(30);
  r_motor.setPWM(-30);
}

void fixHead_lidar(bool left_wall){
  float prevDistance = readLIDAR_sum();
  //const float currentDistance;
  is_turnLeft()? fix_turnLeft() : fix_turnRight();
  delay(40);
  // l_motor.setPWM(0);
  // r_motor.setPWM(0);
  // delay(40);
  while (true) {
    // Read the current distance from the LIDAR sensor
    const float currentDistance = readLIDAR_sum();

    if (currentDistance >= prevDistance) {
      // If the distance is no longer decreasing, stop the motors and exit the loop
      l_motor.setPWM(0);
      r_motor.setPWM(0);
      delay(100);      
      break;
    } else {
      // If the distance is still decreasing, keep turning in the same direction
      is_turnLeft() ? fix_turnLeft() : fix_turnRight();
    }

    prevDistance = currentDistance; // Update the previous distance for the next iteration
    delay(100); // Add a small delay to avoid excessive readings (adjust as needed)
  } 
}

//use ultra sensor to fix
float readUltraSensor(){
  maf_sensor.sample();
  float filtered_dis = maf_sensor.value();
  while (maf_sensor.isReady()) {  //should I do while or if?
    return filtered_dis;
  }
}

void fixHead_ultra(){
  // read sensor, the value should go down when I turn, if not , stop,
  float prevDistance =readUltraSensor();
  is_turnLeft()? fix_turnLeft() : fix_turnRight();// use imu to tell me where should I turn
  delay(40);
  while (true) {
    // Read the current distance from the LIDAR sensor
    const float currentDistance = readUltraSensor();
    if (currentDistance >= prevDistance) {
      // If the distance is no longer decreasing, stop the motors and exit the loop
      l_motor.setPWM(0);
      r_motor.setPWM(0);
      delay(100);      
      break;
    } else {
      // If the distance is still decreasing, keep turning in the same direction
      is_turnLeft() ? fix_turnLeft() : fix_turnRight();
    }
    prevDistance = currentDistance; // Update the previous distance for the next iteration
    delay(100); // Add a small delay to avoid excessive readings (adjust as needed)
  }
}

// use imu to fix head
void fixHead_IMU(){
  while(true){
  int16_t h_imu=imu.getRotationZ();
  if (abs(h_imu-h0)<0.1) {break;}// change this value here
  }
}



void setup() {
  // read the initial heading h0

}

void loop() {
  fixHead();

}
