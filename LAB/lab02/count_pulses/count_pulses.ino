#include <Arduino.h>

#include "Encoder.hpp"

// Encoder pins.
unsigned pin1 = 18;  // This pin gets interrupted.
unsigned pin2 = 22;
uint16_t num_pulses = 0;

void countPulses() { num_pulses++; }

mtrn3100::Encoder l_encoder(pin1, pin2, countPulses);

void setup() { Serial.begin(115200); }

void loop() {
    Serial.println(num_pulses);
    delay(100);
}
