#include <Arduino.h>
#include <ICM_20948.h>
#include "Motor.hpp"
#include "Encoder.hpp"
#include "IMU.hpp"
//#define ICM_20948_USE_DMP

// Set up encoders.
mtrn3100::Motor l_motor(2, 4, 3);
mtrn3100::Motor r_motor(7, 5, 6);
void readLeftEncoder();
void readRightEncoder();
mtrn3100::Encoder l_encoder(18, 22, readLeftEncoder);
mtrn3100::Encoder r_encoder(19, 23, readRightEncoder);
void readLeftEncoder() { l_encoder.readEncoder(); }
void readRightEncoder() { r_encoder.readEncoder(); }

// Set up IMU
mtrn3100::IMU imu(0);


namespace encoder_odometry {
// Pose estimated from encoder odometry.
float x = 0;
float y = 0;
float h = 0;

// COMPLETE THIS BLOCK.
// Robot parameters.
float R = 25;  // mm.
float L = 50;  // mm.

// Last wheel positions.
float lastLPos = 0;
float lastRPos = 0;

void update() {
    const float leftValue = l_encoder.position;
    const float rightValue = r_encoder.position;

    // COMPLETE THIS BLOCK.
    const float tL = leftValue-lastLPos;  // Change in left wheel position.
    const float tR = rightValue-lastRPos;  // Change in right wheel position.
    //const float delta_h = (R/(2*L))*(tR-tL);
    const float delta_s = (R/2)*(tR+tL);
    //float h;
//    if (imu.dataReady()) {
//      //delay(100);
//      imu.read();
//     Serial3.println(imu.yaw());
    //h = imu.yaw()-h0;        
    //}
    //Serial3.println(h);
      x = x + delta_s* cos(h);
    y = y + delta_s* sin(h);

//    delay(1000);
 
    //h = h + delta_h;

    
    lastLPos = leftValue;
    lastRPos = rightValue;

}
}  // namespace encoder_odometry

void setup() {
  
    Serial3.begin(9600);
    //delay(10000);
    Wire.begin();  // This must be called before IMU::begin().
    r_motor.setPWM(0);
    l_motor.setPWM(0);
    r_encoder.position = 0;
    l_encoder.position = 0; 

    imu.begin();
    imu.calibrateAcceleration();
    while (imu.dataReady());
    imu.reset();
}

void loop() {

    //r_motor.setPWM(100);
    //l_motor.setPWM(100);

    // This updates the pose estimated by encoder odometry.
//    encoder_odometry::update();
//
//    // TODO: Print encoder odometry results.
//    Serial.print("[");
//    Serial.print(encoder_odometry::x);
//    Serial.print(",");
//    Serial.print(encoder_odometry::y);
//    Serial.print(",");
//    Serial.print(imu.yaw());
//    Serial.println("]");
    

//     This updates the pose estimated by IMU odometry.
    if (imu.dataReady()) {
        imu.read();
         //TODO: Print encoder odometry results.
//         h = imu.yaw();
        Serial3.println(imu.yaw());
    }
//
//    Serial.println(imu.yaw());
}
