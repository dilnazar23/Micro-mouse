#pragma once

#include <math.h>

namespace mtrn3100 {

class PIDController {
public:
    PIDController(float kp = 1, float ki = 1, float kd = 1, float deadband = 1)
        : kp(kp), ki(ki), kd(kd), deadband(deadband) {}

    float* compute(float error[2],float lidar_error, int wall) {
        //static float output[2];
        const uint32_t curr_time = micros();
        const float dt = static_cast<float>(curr_time - prev_time) / 1e6;
        prev_time = curr_time;
        float l_lidar_err = 0;
        float r_lidar_err = 0;

        //error = setpoint - (input - encoderOffset);
        if (wall == 0){
          error_l = error[0] + lidar_error;
          error_r = error[1] - lidar_error;
        }
        else if (wall == 1){
          error_l = error[0] - lidar_error;
          error_r = error[1] + lidar_error;
        }
        else{
          error_l = error[0];
          error_r = error[1];
        }

        if (fabs(error[0]) < deadband || fabs(error[1]) < deadband) {
            prev_error[0] = 0;
            prev_error[1] = 1;
            output[0] = 0;
            output[1] = 1; 
            return output;
        }
      // For the left motor, comput output
                
        integral[0] = integral[0] + error_l*dt;    
        derivative[0] = (error_l- prev_error[0])/dt;  
        output[0] = kp*error_l + ki*integral[0] + kd* derivative[0];      

        prev_integral[0] = integral[0];
        prev_error[0] = error_l;
        
       // For the right motor, compute output 
        
        integral[1] = integral[1] + error_r*dt;    
        derivative[1] = (error_r- prev_error[1])/dt;  
        output[1] = kp*error_r + ki*integral[1] + kd* derivative[1];  

        prev_integral[1] = integral[1];
        prev_error[1] = error_r;

        return output;
    }

        void tune(float p, float i, float d) {
        kp = p;
        ki = i;
        kd = d;
    }

    void setDeadband(float limit) { deadband = limit; }

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
    float error_l = 0;
    float error_r = 0;
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