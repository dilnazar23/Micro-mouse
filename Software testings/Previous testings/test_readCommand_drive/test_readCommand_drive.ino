//Read command by bluetooth function
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
constexpr uint8_t pwm = 100;
constexpr uint16_t forwardDuration = 500;
constexpr uint16_t turnDuration = 300;



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

String ReadCommand(){
    Serial3.println("Enter command:");
    while(Serial3.read() == -1){};        
    if (Serial.available() > 0 ) {
        Serial3.write(Serial.read());
        Serial.print("Read it");
    }
    if (Serial3.available()) {
        Serial.write(Serial3.read());
        Serial3.print("Read it");    
    }
    //};
    Serial3.println("Command received");
    return (Serial3.readString());
}

// Drive by command function
void drive_by_command(){
    String command = ReadCommand();
    
    switch (command[0]) {
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

}

void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);
  l_motor.setPWM(0);
  r_motor.setPWM(0);
  delay(200);
  l_encoder.position = 0;
  r_encoder.position = 0;
}

void loop() {
  drive_by_command();
}
