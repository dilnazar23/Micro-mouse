#include <Arduino.h>
//#include <PID_v1.h>
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

// Set up PID
mtrn3100::PIDController pid(38, 0, 0.2, 0);

 //PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
// Drive parameters.
int pwm = 100;

// Rename tuple to more readable datatype name.
using MazeCellPose = mtrn3100::Tuple<float, float, float>;

// Determines if the movement between two poses is purely translational or rotational. Returns true for translational.
bool isTranslationalMovement(MazeCellPose const& from, MazeCellPose const& to) {
    return mtrn3100::get<2>(from) == mtrn3100::get<2>(to);
}

// Pose sequence plan.
constexpr size_t poseSequenceLength = 3;
size_t poseIndex = 0;
// const MazeCellPose poseSequence[poseSequenceLength] = {{0, 0, 0},      {100, 0, 0},  {100, 0, M_PI / 2},
//                                                        {100, 0, M_PI}, {0, 0, M_PI}};
//just forward
const MazeCellPose poseSequence[poseSequenceLength] = {{0, 0, 0},  {0, 0, M_PI / 2},{0, 0, M_PI }};

namespace inverse_kinematics {

// Current pose.
float x0 = 0;  // mm.
float y0 = 0;  // mm.
float h0 = 0;  // rad.

// Robot parameters.
float R = 23;   // mm.
float L = 140;  // mm.

// Compute left and right wheel position changes for pure translational movement.
mtrn3100::Tuple<float, float> linear(float x1, float y1) {
    // COMPLETE THIS FUNCTION.
    float thetaL = static_cast<float>(sqrt(sq(x1-x0)+sq(y1-y0)))/R ;
    float thetaR = thetaL ;
    mtrn3100::Tuple<float,float> wheel_position = {thetaL,thetaR};
    return wheel_position;
}

// Compute left and right wheel position changes for pure rotational movement.
mtrn3100::Tuple<float, float> rotational(float h1) {
    // COMPLETE THIS FUNCTION.
    float thetaL = abs((h1-h0)*(L/2)/R );
    float thetaR = -thetaL ;
    mtrn3100::Tuple<float,float> wheel_position = {thetaL,thetaR};
    return wheel_position;
}

// Updates the current pose with the next pose.
void update(float x1, float y1, float h1) {
    x0 = x1;
    y0 = y1;
    h0 = h1;
}

}  // namespace inverse_kinematics


template <typename T>
int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

void setup() {
    Serial.begin(9600);
    Serial3.begin(9600);
    l_motor.setPWM(0);
    r_motor.setPWM(0);
    delay(500);
    l_encoder.position = 0;
    l_encoder.position = 0;
}
// float error_l =100000;
// float error_r  =100000;
//float l_pid_speed = 0;
//float r_pid_speed = 0;
void loop() {
    const float thetaL = l_encoder.position;
    const float thetaR = r_encoder.position;
    // Determine is movement is translational or rotational.
    const bool isTranslation = isTranslationalMovement(poseSequence[poseIndex], poseSequence[poseIndex + 1]);
    const auto nextPose = poseSequence[++poseIndex];
    const float x1 = mtrn3100::get<0>(nextPose);
    const float y1 = mtrn3100::get<1>(nextPose);
    const float h1 = mtrn3100::get<2>(nextPose);
    mtrn3100::Tuple<float,float> wheel_position (0,0);
    // Get wheel position changes.
    if (isTranslation) {
      wheel_position= inverse_kinematics::linear(x1,y1);
    }
    else{
     wheel_position= inverse_kinematics::rotational(h1);
    };

    float l_expect = mtrn3100::get<0>(wheel_position);
    float r_expect = mtrn3100::get<1>(wheel_position);
       
    //Debugging
    //  float speed = 10.0001;
    //  //float speed = 10.0001;
    // Serial.println(speed);
    // l_motor.setPWM(speed);
    // r_motor.setPWM(speed);
    // while (1){};

    float error_l = l_expect - (l_encoder.position-thetaL);
    float error_r = r_expect - (r_encoder.position-thetaR);
    float errors[2] = {error_l,error_r};
    Serial.print(error_l);
    Serial.print("  ");
    Serial.println(error_r);
    // float* pid_result;
    // pid_result = pid.compute(errors);
    // Serial.print(pid_result[0]);
    //while(1){};
    // error_l = l_expect - (l_encoder.position-thetaL);
    // error_r = r_expect - (r_encoder.position-thetaR);
    // errors[0] = error_l;
    // errors[1] = error_r;
    
    //while(1){}
    //l_motor.setPWM(l_pid_speed);
    //r_motor.setPWM(r_pid_speed);

    //while(1){}
    while((fabs(error_l) >=0.1) || (fabs(error_r) >=0.1)){
      error_l = l_expect - (l_encoder.position-thetaL);
      error_r = r_expect - (r_encoder.position-thetaR);
      errors[0] = error_l;
      errors[1] = error_r;
      float* pid_result = pid.compute(errors);
      float l_pid_speed = pid_result[0];
      float r_pid_speed = pid_result[1];
      //Serial.println(l_pid_speed);
      if (fabs(error_l) >=0.1) l_motor.setPWM(l_pid_speed);
      else l_motor.setPWM(0);
      if (fabs(error_r) >=0.1) r_motor.setPWM(r_pid_speed);
      else r_motor.setPWM(0);
      Serial3.println(l_pid_speed);
      Serial3.println(r_pid_speed);
      Serial3.println(error_l);
      Serial3.println(error_r);
    }
    // while ((error_l >=0.1) || (error_r>=0.1 )) {
    //   //Serial.print("I'm here");
    //   error_l = l_expect - (thetaL-l_encoder.position);
    //   error_r = r_expect - (thetaR-r_encoder.position);
    //   errors[0] = error_l;
    //   errors[1] = error_r;
    //   const float* pid_result = pid.compute(errors);
    //   const float l_pid_speed = pid_result[0];
    //   const float r_pid_speed = pid_result[1];
    //   // Serial.println(l_pid_speed);
    //   // while(1){}
    //   l_motor.setPWM(l_pid_speed);
    //   r_motor.setPWM(r_pid_speed);

    //   //while(1){}
      
    //   //Serial.print("I'm There");
    // }

    // Set motors to 0 when wheel position has been reached.
    l_motor.setPWM(0);
    r_motor.setPWM(0);

    Serial3.print("one command done");
    delay(10);

    // const float cur_motor_pos = 0;
    // r_encoder.position = 0;
    
    inverse_kinematics::update(x1,y1,h1);
    // So we can see when a movement has ended.
    delay(1000);
    

    // Stop program if drive plan is complete.
    while (poseIndex == poseSequenceLength - 1) {      
    }
}
