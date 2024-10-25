#include <Arduino.h>

#include "Encoder.hpp"
#include "Motor.hpp"

// Motor pins.
mtrn3100::Motor l_motor(2, 4, 3);
mtrn3100::Motor r_motor(7, 5, 6);

// Encoder pins.
void readLeftEncoder();
void readRightEncoder();
mtrn3100::Encoder l_encoder(18, 22, readLeftEncoder);
mtrn3100::Encoder r_encoder(19, 23, readRightEncoder);
void readLeftEncoder() { l_encoder.readEncoder(); }
void readRightEncoder() { r_encoder.readEncoder(); }

// Used for recording and graphing the motor position.
constexpr unsigned sampling_rate = 0.472;  // COMPLETE THIS LINE.
constexpr uint16_t max_samples = 260;    // COMPLETE THIS LINE.
float positions[max_samples];
//unsigned long startMillis;
//unsigned long currentMillis;
//const unsigned long period = 1000;
// COMPLETE THIS FUNCTION.
void saveSamples() {
  delay(4);
  int size= sizeof(positions)/ sizeof(int); //8
  positions[size+1]=r_encoder.position;
  }
    

// COMPLETE THIS FUNCTION.
void plotMotorState() {
  for (int i = 0; i++; i < sizeof(positions)) {
  Serial.println(positions[i]);
  }
}

void setup() { Serial.begin(115200); }

void loop() {
    // This block of code is an example of how to drive the motor.
    //r_motor.setPWM(200);
    //Serial.println("stop");
    
    
    
    l_motor.setPWM(100);
    r_motor.setPWM(100);
    delay(10000);
    l_motor.setPWM(0);
    //_motor.setPWM(0);
    // Infinite loop to stop the sketch.
    while (1) {
    }
}
