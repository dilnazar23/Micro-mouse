#pragma once

#include <math.h>

namespace mtrn3100 {

class PIDController {
public:
    PIDController(float kp = 1, float ki = 1, float kd = 1, float deadband = 1)
        : kp(kp), ki(ki), kd(kd), deadband(deadband) {}

    float compute(float error) {
        const uint32_t curr_time = micros();
        const float dt = static_cast<float>(curr_time - prev_time) / 1e6;
        prev_time = curr_time;

        //error = setpoint - (input - encoderOffset);

        if (fabs(error) < deadband) {
            prev_error = 0;
            return 0;
        }

        integral = integral + error;    // COMPLETE THIS LINE.
        derivative = error- prev_error;  // COMPLETE THIS LINE.
        output = kp*error + ki*integral + kd* derivative/dt;      // COMPLETE THIS LINE.

        prev_integral = integral;
        prev_error = error;

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
    float error;
    float derivative;
    float integral;
    float output;
    float prev_integral = 0;
    float prev_error = 0;
    float setpoint = 0;
    float encoderOffset = 0;
    uint32_t prev_time = micros();
};

}  // namespace mtrn3100