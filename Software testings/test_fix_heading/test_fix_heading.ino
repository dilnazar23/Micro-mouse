#include <math.h>
// GET THE PATH(sets of corrdinates) VIA BLUETOOTH
// RUN with pid on encoder till 250(need to test how long) left from the goal，or too close to wall
// set the upper limit speed very low
// adjust position to center by lidar(循迹小车逻辑)-->stop till
//安全急刹
// adjust heading
#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <VL6180X.h>
#include "pid_controller.hpp"
#include "Tuple.hpp"
//#include "Robot.hpp"
#include "Encoder.hpp"
#include "Motor.hpp"
//#include "LCD.hpp"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "MAFUltrasonicSensor.hpp"
#include "UltrasonicSensor.hpp"
mtrn3100::PIDController pid(9, 0, 0, 0);
//DECLARE GLOBAL ROBOT PARAMETER
float lastLPos,lastRPos;
float x0,y0,h0,x,y,h,x1,y1,h1;
const float L=140,R=23;
// Set up motors.
mtrn3100::Motor r_motor(2, 4, 3);
mtrn3100::Motor l_motor(7, 5, 6);

// Set up encoders.
void readLeftEncoder();
void readRightEncoder();
mtrn3100::Encoder l_encoder(18, 22, readLeftEncoder);
mtrn3100::Encoder r_encoder(19, 23, readRightEncoder);
void readLeftEncoder() { l_encoder.readEncoder(); }
void readRightEncoder() { r_encoder.readEncoder(); }

// Set up Lidar
#define RANGE 1
#define address0 0x20
#define address1 0x22
int enablePin0 = 11;
int enablePin1 = 10;
VL6180X sensor0;
VL6180X sensor1;

// Set up IMU
MPU6050 imu;

// Set up Sensor
mtrn3100::UltrasonicSensor sensor(40, 38);
mtrn3100::MAFUltrasonicSensor<10> maf_sensor(sensor);

// FIX HEADING
//--- DETECT WALL-> IF NO WALL-> FIX WITH IMU
//----------------- IF L/R WALL-> FIX WITH LIDAR
//----------------- IF F WALL-> FIX WITH SENSOR
// namespace FIXHEAD{
void fixHead(bool leftwall){
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
      fixHead_imu();
      Serial.println("Im correct heading");
      break;
  }
}
char detectWall() {
  float front;
  float left;
  float right;
  // this is without MAF 
  //maf_sensor.sample();
  front = sensor.echo();
  // if (maf_sensor.isReady()) {
  //   front = filtered_dis;
  // }
  //   delay(100);

  left = sensor0.readRangeContinuousMillimeters();
  right = sensor1.readRangeContinuousMillimeters();

    
  if (front < 150) {return 'F';}
  if (left < 150) {return 'L';}
  if (right < 150) {return 'R';}
}

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

void fixHead_lidar(bool leftwall){
  float prevDistance = readLIDAR_sum();
  //const float currentDistance;
  Serial.println(prevDistance);
  fix_turnLeft();
  delay(10);
  while (true) {
    // Read the current distance from the LIDAR sensor
    const float currentDistance = readLIDAR_sum();
    Serial.println(currentDistance);
    if (currentDistance -prevDistance>=3) {
      // If the distance is no longer decreasing, stop the motors and exit the loop
      l_motor.setPWM(0);
      r_motor.setPWM(0);
      delay(100);      
      break;
    } else {
      // If the distance is still decreasing, keep turning in the same direction
      fix_turnLeft();
      delay(10);
    }
    prevDistance = currentDistance; // Update the previous distance for the next iteration
    //delay(100); // Add a small delay to avoid excessive readings (adjust as needed)
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
    const float currentDistance = sensor0.readRangeContinuousMillimeters()+sensor1.readRangeContinuousMillimeters(); //idk is this the LIDAR sensor distance we want
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
void fixHead_imu(){
  while(1){
    const int TEST fabs(imu.getYaw()-h0)
    Serial.print(TEST);
    }
  while(fabs(imu.getYaw()-h0)>1){
  if (isTurnLeft()){}//turn left}
  else()//turn right

  }
}



void setup() {
  //------SET UP SERIAL
  //------SET UP HARDWARRE------//
  // BLUETOOTH
  // MOTOR
  // ENCODER
  // LIDAR
  // --READ INITIAL LIDAR READING
  // ULTRA SENSOR
  // IMU
  // --READ INITIAL IMU HEADING 
  // GET THE PATH_CELL FROM BLUETOOTH
  // CONVERT PATH_CELL TO PATH_POS
  // INITIALISE POS
  // FIX INITIAL HEADING
  // SEND TO ME "READY TO RUN"
  // DELAY
    //Set up serial and wire and bluetooth
    Serial.begin(9600);
    // Serial3.begin(9600);
    Wire.begin();
    //Set up motor
    r_motor.setPWM(0);
    l_motor.setPWM(0);
    delay(500);
    //Set up Encoder
    r_encoder.position = 0;
    l_encoder.position = 0; 
    //Set up Robot
//     robot.x = robot.x0;
//     robot.y = robot.y0;
//     robot.h = robot.h0;
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
    delay(3000);

    // set up IMU 
    Serial.println(F("Initializing I2C devices..."));
    imu.initialize();
    
    Serial.println(F("Testing device connections..."));
    Serial.println(imu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    Serial.println("IMU ready! Start reading in 3 seconds ...!");
    delay(3000);

    // initial positions
    x0=0;y0=0;h0=0;
    x1=0;y1=0;h1=M_PI/2;

    // h1 is what we want 
    // h0 is our initial 
    // getrotationZ,  is our current heading we want 


}

void loop() {
  // FIX HEADING BEFORE RUN
  // GET NEXT POS
  // CHECK LINEAR OR ROTATION
  // IF ROTATION--> ROTATION
  // IF LINEAR
  // -> RUN WITH ENCODER PID--> STOP THIS FUNCTION TILL CLOSE TO GOAL OR CLOSE TO WALL
  // -> PARKING --> STOP UNTILL ROBOT IN THE MIDDLE OR GET THE GOAL

  // FINISH AND STOP
  //UPDATE TARGET POS
  fixHead_lidar(true);
  while(1){}
  // detectWall();
  // Serial.println(imu.getRotationZ()*M_PI/180);
  // delay(100);
  
}
