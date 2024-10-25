#include <Arduino.h>

#include "UltrasonicSensor.hpp"

mtrn3100::UltrasonicSensor ultrasonic_sensor(40,38);

void setup() { Serial.begin(9600); 
               //digitalWrite(42,)
}

void loop() {
    //delay(100);
    Serial.print("Ultrasonic:");
    Serial.println(ultrasonic_sensor.echo());
}
