#include <Arduino.h>

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

// Drive parameters.
int pwm = 500;

// Rename tuple to more readable datatype name.
using MazeCellPose = mtrn3100::Tuple<float, float, float>;

// Determines if the movement between two poses is purely translational or rotational. Returns true for translational.
bool isTranslationalMovement(MazeCellPose const& from, MazeCellPose const& to) {
    return mtrn3100::get<2>(from) == mtrn3100::get<2>(to);
}

// Pose sequence plan.
constexpr size_t poseSequenceLength = 6;
size_t poseIndex = 1;
const MazeCellPose poseSequence[poseSequenceLength] = {{0, 0, 0},      {100, 0, 0},  {100, 0, M_PI / 2},
                                                       {100, 0, M_PI}, {0, 0, M_PI}, {0, 0, M_PI / 2}};

namespace inverse_kinematics {

// Current pose.
float x0 = 0;  // mm.
float y0 = 0;  // mm.
float h0 = 0;  // rad.

// Robot parameters.
float R = 23;   // mm.
float L = 130;  // mm.

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
    float thetaL = abs((h1-h0)*(L/2)/R) ;
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

// Helper function which returns the sign of a value: -1 for negative, 0, or 1 for positive.
//
// Author: user79758
// Link: https://stackoverflow.com/a/4609795
// License: https://creativecommons.org/licenses/by-sa/4.0/
// Changes:
template <typename T>
int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

void setup() {
    Serial.begin(9600);
    l_motor.setPWM(0);
    r_motor.setPWM(0);
    delay(200);
    l_encoder.position = 0;
    l_encoder.position = 0;
}

void loop() {
    // COMPLETE THIS LOOP.
    // The following loop has been partially done for you.
    // Follow the comments which helps to keep the structure of the control loop.
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
      // l_motor.setPWM(pwm);
      // r_motor.setPWM(pwm);      
    }
    else{
     wheel_position= inverse_kinematics::rotational(h1);
    };
    
    // Update global variables with expected next pose instead of actual next pose.    
    // Drive motors and accounting for signum of wheel changes for direction.
    float l_expect = mtrn3100::get<0>(wheel_position);
    float r_expect = mtrn3100::get<1>(wheel_position);
    // Serial.println(sign(l_expect)*pwm);
    // Serial.println(sign(r_expect)*pwm);
    // while(1);
    l_motor.setPWM(sign(l_expect)*pwm);
    r_motor.setPWM(sign(r_expect)*pwm);
    while (!(abs(l_expect - abs(thetaL-l_encoder.position)) <= 0.1)&& !(abs(r_expect - abs(thetaR-r_encoder.position)) <= 0.1)){
        // if (sign(l_expect) != sign(thetaL)) {
        //   l_motor.setPWM(-pwm);
        // } else {
        //   l_motor.setPWM(pwm);

        // }
        // if (sign(r_expect) != sign(thetaR)) {
        //   r_motor.setPWM(-pwm);
        // } else {
        //   r_motor.setPWM(pwm);

        // }
        Serial.print(mtrn3100::get<0>(wheel_position));
        Serial.print(",");
        Serial.print(mtrn3100::get<1>(wheel_position));
        // Serial.print(",");
        // Serial.print(l_expect);
        // Serial.print(",");
        // Serial.print(r_expect);
        Serial.print(",");
        Serial.print(abs(mtrn3100::get<0>(wheel_position) - l_encoder.position)); 
        Serial.print(",");
        Serial.print(abs(mtrn3100::get<1>(wheel_position) - r_encoder.position));  
        Serial.println();
    };
    // Keep driving until left and/or right wheel position has been reached.
    // You could try checking if the difference between expected and actual is close to 0.
    // You could also try by using the signum, check if the actual value went beyond the expected.
    while (false) {
        // Don't need to do anything in the while loop if open-loop control.
        delay(1);
    }

    // Set motors to 0 when wheel position has been reached.
    l_motor.setPWM(0);
    r_motor.setPWM(0);

    Serial.print("one command done");
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
