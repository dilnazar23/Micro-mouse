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
//#include "IMU.hpp"
#include "MAFUltrasonicSensor.hpp"
#include "UltrasonicSensor.hpp"
mtrn3100::PIDController pid(8, 0, 0, 0);

float x0=0,y0=0,h0=M_PI/2,x1=0,y1=1000,h1=M_PI/2;
float x=0,y=0,h=0;
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

namespace inverse_kinematics {

// Compute left and right wheel position changes for pure translational movement.
mtrn3100::Tuple<float, float> linear(float x1, float y1) {  
  float thetaL = static_cast<float>(sqrt(sq(x1 - x0) + sq(y1 - y0))) / R;
  float thetaR = thetaL;
  mtrn3100::Tuple<float, float> wheel_position = { thetaL, thetaR };
  return wheel_position;
}

// Compute left and right wheel position changes for pure rotational movement.
mtrn3100::Tuple<float, float> rotational(float h1) {
  // COMPLETE THIS FUNCTION.
  float thetaL = (h1 - h0) * (L / 2) / R;
  float thetaR = -thetaL;
  mtrn3100::Tuple<float, float> wheel_position = { thetaL, thetaR };
  return wheel_position;
}
// Updates the current pose with the next pose.
void tarPosUpdate(float x1, float y1, float h1) {
  x0 = x1;
  y0 = y1;
  h0 = h1;
}

}  // namespace inverse_kinematics

//namespace ODOMETRY{

float lastLPos,lastRPos;

void pos_init(){ //need to modofy as read the initial pose from bluetooth or pose sequence
  //  READ THE FIRST FIRST CELL AND CONVERT TO INITIAL POS
  x = x0;
  y = y0;
  h = h0;
// Last wheel positions.
  lastLPos = 0;
  lastRPos = 0;
}
void  posUpdate(){    
    const float leftValue = l_encoder.position;
    const float rightValue = r_encoder.position;    
    float dL = R*(leftValue-lastLPos);  // distance travel by the left wheel
    float dR = R*(rightValue-lastRPos);  // distance travel by the right wheel
    float d = (dL+dR)/2; // distance travel by the center of robot
    float delta_h = (dR-dL)/L; // change of heading
    // update x,y,h base on the change
    x = x + d * cos(h+(delta_h/2));
    y = y + d * sin(h+(delta_h/2));
    h = h + delta_h;
    //update the wheel position
    lastLPos = leftValue;
    lastRPos = rightValue; 

}
// CHECK IF I GET TO GOAL
//--- CHECK X Y
bool getGoal(){ // 
  return (abs(x-x1)<5 && abs(y-y1)<5);}
//} // namespace ODOMETRY

void forwardEncoder(){
  const float thetaL = l_encoder.position;
  const float thetaR = r_encoder.position;
  mtrn3100::Tuple<float, float> wheel_position(0, 0);
  wheel_position = inverse_kinematics::linear(x1, y1);
  float l_expect = mtrn3100::get<0>(wheel_position);
  float r_expect = mtrn3100::get<1>(wheel_position);
  float error_l = l_expect - (l_encoder.position - thetaL);
  float error_r = r_expect - (r_encoder.position - thetaR);
  float errors[2] = { error_l, error_r };
  while((fabs(error_l) >=0.03) || (fabs(error_r) >=0.03)){
      error_l = l_expect - (l_encoder.position-thetaL);
      error_r = r_expect - (r_encoder.position-thetaR);
      errors[0] = error_l;
      errors[1] = error_r;
      float* pid_result = pid.compute(errors);
      float l_pid_speed = pid_result[0];
      float r_pid_speed = pid_result[1];
      //Serial.println(l_pid_speed);
      if (fabs(error_l) >0.03){ 
        l_motor.setPWM(l_pid_speed);      
        }
      else {
        l_motor.setPWM(0);
        delay(10);
        pid.resetIntegral(1);
        };

      if (fabs(error_r) >=0.03) r_motor.setPWM(r_pid_speed);
      else {
        r_motor.setPWM(0);
        delay(10);
        pid.resetIntegral(0);
        };
      Serial.print("l_pid speed: ");
      Serial.println(l_pid_speed);
      Serial.print("r_pid speed: ");
      Serial.println(r_pid_speed);
      Serial.print("l_error: ");
      Serial.println(error_l);
      Serial.print("r_error: ");
      Serial.println(error_r);
    }
}
void setup() {
    Serial.begin(9600);
    Serial3.begin(9600);
    l_motor.setPWM(0);
    r_motor.setPWM(0);
    delay(200);
    l_encoder.position = 0;
    l_encoder.position = 0;    
}

void loop() {

  pos_init();
  forwardEncoder();
  l_motor.setPWM(0);
  r_motor.setPWM(0);
  posUpdate();
  Serial.print("x:");
  Serial.print(x);
  Serial.print("y:");
  Serial.print(y);
  Serial.print("h:");
  Serial.println(h);
  while(1){}
}
