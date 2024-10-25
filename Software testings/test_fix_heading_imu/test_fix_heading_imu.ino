// GET THE PATH(sets of corrdinates) VIA BLUETOOTH
// RUN with pid on encoder till 250(need to test how long) left from the goal，or too close to wall
// set the upper limit speed very low
// adjust position to center by lidar(循迹小车逻辑)-->stop till
//安全急刹
// adjust heading
#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Arduino.h>
#include <VL6180X.h>
#include "pid_controller.hpp"
#include "Tuple.hpp"
#include "Encoder.hpp"
#include "Motor.hpp"
#include "MPU6050.hpp"
mtrn3100::PIDController pid(9, 0, 0, 0);
//DECLARE GLOBAL ROBOT PARAMETER

float x0,y0,h0,x,y,h,x1,y1,h1;

const float L=130,R=23;
// Set up motors.
mtrn3100::Motor r_motor(2, 4, 3);
mtrn3100::Motor l_motor(7, 5, 6);

// Set up encoders.
void readLeftEncoder();
void readRightEncoder();
mtrn3100::Encoder r_encoder(18, 22, readRightEncoder);
mtrn3100::Encoder l_encoder(19, 23, readLeftEncoder);
void readLeftEncoder() { l_encoder.readEncoder(); }
void readRightEncoder() { r_encoder.readEncoder(); }

// Set up IMU
IMU imu;

// FIX HEADING
//--- DETECT WALL-> IF NO WALL-> FIX WITH IMU
//----------------- IF L/R WALL-> FIX WITH LIDAR
//----------------- IF F WALL-> FIX WITH SENSOR
// namespace FIXHEAD{



bool is_turnLeft(){     // use imu to tell me where should i turn to fix head
  int16_t h_imu=imu.getYaw();//read imu  
  if (h_imu<h0){ return true;} 
  else {return false;}
}
// FIX HEADING WITH LIDAR
//--- READ IMU
//--- CHECK TURN LEFT OR STOP
//--- TURN UNTILL LIDAR READING GO UP
//--- REPEAT TILL LIDAR READING GOOD

void fix_turnLeft(){
  l_motor.setPWM(32);
  r_motor.setPWM(-32);
}
void fix_turnRight(){
  l_motor.setPWM(-32);
  r_motor.setPWM(32);
}
// use imu to fix head
void fixHead_imu(){
  while((imu.getYaw()-h0)>5){fix_turnLeft();}
  while((imu.getYaw()-h0)<-5){fix_turnRight();}
  l_motor.setPWM(0);
  r_motor.setPWM(0);
}
void setup() {
    //---Communication INITIALISE----
  Wire.begin();
  Serial.begin(9600);
  Serial3.begin(9600); 

  //----HARDWARE INITIALISE----  
  //Set up motor
  r_motor.setPWM(0);
  l_motor.setPWM(0);
  delay(100);
  //Set up Encoder
  r_encoder.position = 0;
  l_encoder.position = 0;
  //Set up IMU
  // Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  // Serial.begin(9600);
  h0=0;

}

void loop() {
  // Serial.println(imu.getYaw());
  fixHead_imu();
  // while(1){}
  
}
