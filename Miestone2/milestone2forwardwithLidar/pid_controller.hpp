#pragma once

#include <math.h>

namespace mtrn3100 {

class PIDController {
public:
    PIDController(float kp = 1, float ki = 1, float kd = 1, float deadband = 1)
        : kp(kp), ki(ki), kd(kd), deadband(deadband) {}

    float* compute(float error[2]) {
        //static float output[2];
        const uint32_t curr_time = micros();
        const float dt = static_cast<float>(curr_time - prev_time) / 1e6;
        prev_time = curr_time;

        //error = setpoint - (input - encoderOffset);

        if (fabs(error[0]) < deadband || fabs(error[1]) < deadband) {
            prev_error[0] = 0;
            prev_error[1] = 1;
            output[0] = 0;
            output[1] = 1; 
            return output;
        }

        integral[0] = prev_integral[0] + error[0]*dt;    // COMPLETE THIS LINE.
        derivative[0] = (error[0]- prev_error[0])/dt;  // COMPLETE THIS LINE.
        output[0] = kp*error[0] + (ki+0.05)*integral[0] + (kd+0.06)* derivative[0];      // COMPLETE THIS LINE.

        prev_integral[0] = integral[0];
        prev_error[0] = error[0];

        integral[1] = prev_integral[1] + error[1]*dt;    // COMPLETE THIS LINE.
        derivative[1] = (error[1]- prev_error[1])/dt;  // COMPLETE THIS LINE.
        output[1] = kp*error[1] + ki*integral[1] + kd* derivative[1];      // COMPLETE THIS LINE.

        prev_integral[1] = integral[1];
        prev_error[1] = error[1];


        return output;
    }

        void tune(float p, float i, float d) {
        kp = p;
        ki = i;
        kd = d;
    }

    void resetIntegral(bool isLeft) {
      if(isLeft){prev_integral[0] = 0;}
      else{prev_integral[1] = 0;}
       }

    // void zeroEncoderAndSetTarget(float currentEncoderCount, float setpointInput) {
    //     encoderOffset = currentEncoderCount;
    //     setpoint = setpointInput;
    // }

private:
    float kp;
    float ki;
    float kd;
    float deadband;
    float error[2] = {0,0};
    float derivative[2] = {0,0};
    float integral[2] = {0,0};
    //int* result = new int[size];
    float* output =new float[2];
    float prev_integral[2] = {0,0};
    float prev_error[2] = {0,0};
    float setpoint = 0;
    float encoderOffset = 0;
    uint32_t prev_time = micros();
};

}  // namespace mtrn3100