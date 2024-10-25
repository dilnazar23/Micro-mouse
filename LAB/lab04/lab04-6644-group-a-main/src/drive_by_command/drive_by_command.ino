#include <Arduino.h>
#include "Encoder.hpp"
#include "Motor.hpp"

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
constexpr uint8_t pwm = 80;
constexpr uint16_t forwardDuration = 10000;
constexpr uint16_t turnDuration = 300;

// Drive plan.
constexpr size_t drivePlanLength = 5;
char drivePlan[drivePlanLength + 1] = "FLRLR";
size_t drivePlanIndex = 0;

void forward() {
    l_motor.setPWM(pwm);
    r_motor.setPWM(pwm);
    delay(forwardDuration);
    l_motor.setPWM(0);
    r_motor.setPWM(0);
    delay(500);
}

void turnLeft() {
    l_motor.setPWM(-pwm);
    r_motor.setPWM(pwm);
    delay(turnDuration);
    l_motor.setPWM(0);
    r_motor.setPWM(0);
    delay(500);
}

void turnRight() {
    l_motor.setPWM(pwm);
    r_motor.setPWM(-pwm);
    delay(turnDuration);
    l_motor.setPWM(0);
    r_motor.setPWM(0);
    delay(500);
}

void setup() {
    Serial.begin(9600);
    delay(5000);
    Serial.print("Ready");
}

void loop() {
    // Get next motion.
    l_motor.setPWM(0);
    r_motor.setPWM(0);
    switch (drivePlan[drivePlanIndex]) {
        case 'L':
            turnLeft();
            break;
        case 'R':
            turnRight();
            break;
        case 'F':
            forward();
            break;
    }

    drivePlanIndex++;
    Serial.print(l_encoder.position);
    Serial.print(",");
    Serial.print(r_encoder.position);
    Serial.println();
   //  Stop program if drive plan is complete.
    while (drivePlanIndex == drivePlanLength) {
      Serial.print(0);
      l_motor.setPWM(0);
      delay(1000000);
      Serial.print(1);
      r_motor.setPWM(0);
    }
}
