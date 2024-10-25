#include <Arduino.h>

#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"

// Motor pins.
mtrn3100::Motor l_motor(2, 4, 3);
mtrn3100::Motor r_motor(7, 5, 6);

// Set up PIDs.
mtrn3100::PIDController l_pos_pid(70, 0, 50, 0.2);
mtrn3100::PIDController r_pos_pid(70, 0, 50, 0.2);

// Encoder pins.
void readLeftEncoder();
void readRightEncoder();
mtrn3100::Encoder l_encoder(18, 22, readLeftEncoder);
mtrn3100::Encoder r_encoder(19, 23, readRightEncoder);
void readLeftEncoder() { l_encoder.readEncoder(); }
void readRightEncoder() { r_encoder.readEncoder(); }

// Global variables.
const float rev = 2 * M_PI;
float target = 0;

float leftMotorsignal;
float rightMotorsignal;

String incomingByte;

void setup() {
    Serial.begin(115200);
    l_pos_pid.zeroEncoderAndSetTarget(l_encoder.position, target);
    r_pos_pid.zeroEncoderAndSetTarget(r_encoder.position, target);
    Serial.println("ENTER THE NUMBER OF RADIANS TO MOVE THE WHEELS");
    Serial.println("INPUT CAN BE POSITIVE OR NEGATIVE BUT MUST BE A VALID FLOAT");
}

void loop() {
    // Read New Desired position From Serial Monitor
    if (Serial.available() > 0) {
        incomingByte = Serial.readString();
        target = atof(incomingByte.c_str());

        l_pos_pid.zeroEncoderAndSetTarget(l_encoder.position, target);
        r_pos_pid.zeroEncoderAndSetTarget(r_encoder.position, target);
    }

    // PID.
    leftMotorsignal = l_pos_pid.compute(l_encoder.position);
    rightMotorsignal = r_pos_pid.compute(r_encoder.position);
    l_motor.setPWM(leftMotorsignal);
    r_motor.setPWM(rightMotorsignal);
}
