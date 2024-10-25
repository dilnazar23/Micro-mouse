#pragma once

#include <Arduino.h>

namespace THE_ENGINEERS {
class Robot {
    public:
        Robot(float Length, float Radius) : L(Length),R(Radius) {            
            };
        float x;
        float y;
        float h;
        char displayHead;
        float w;
        float v_x;
        float v_y;
        float v;
        float distanceLeft;
        float distanceFront;
        float distanceRight;

        float leftWheelSpeed;
        float rightWheelSpeed;

        float x0;
        float y0;
        float h0;
        int walls[3] = {0,0,0};

    //     void forward() {
    //         l_motor.setPWM(pwm);
    //         r_motor.setPWM(pwm);
    //         delay(forwardDuration);
    //         l_motor.setPWM(0);
    //         r_motor.setPWM(0);
    //         delay(500);
    //     }

    //     void turnLeft() {
    //         l_motor.setPWM(-pwm);
    //         r_motor.setPWM(pwm);
    //         delay(turnDuration);
    //         l_motor.setPWM(0);
    //         r_motor.setPWM(0);
    //         delay(500);
    //     }

    //     void turnRight() {
    //         l_motor.setPWM(pwm);
    //         r_motor.setPWM(-pwm);
    //         delay(turnDuration);
    //         l_motor.setPWM(0);
    //         r_motor.setPWM(0);
    //         delay(500);
    //     }
    //     void drive_by_command(){
    //         switch (drivePlan[drivePlanIndex]) {
    //     case 'L':
    //         turnLeft();
    //         break;
    //     case 'R':
    //         turnRight();
    //         break;
    //     case 'F':
    //         forward();
    //         break;
    // }

    // drivePlanIndex++;

    // // Stop program if drive plan is complete.
    // while (drivePlanIndex == drivePlanLength) {
    // }
    //     };

//        void update(){};
//        float get_v(){return v};
//        float get_vx(){return v_x};
//        float get_vy(){return v_y};
//
//        void getBluetoothStatus ();
//        void getMotorStatus ();
//        void getLCDStatus ();
//        void getLIDARStatus ();
//        void getUltrasonicStatus ();
//        void getIMUStatus ();
    private:
    const uint8_t L;
    const uint8_t R;
        
};
}//NAME SPACE:THE_ENGINEERS
