#pragma once

#include <Arduino.h>

namespace mtrn3100 {

class Encoder {
public:
    Encoder(uint8_t enc1, uint8_t enc2, void* callback) : encoder1_pin(enc1), encoder2_pin(enc2) {
        pinMode(encoder1_pin, INPUT_PULLUP);
        pinMode(encoder2_pin, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(encoder1_pin), callback, RISING);
    }
    int num_pulses=0;
    //void countPulses() { num_pulses++; }
    void readEncoder() {
        num_pulses++;
        noInterrupts();
        if (digitalRead(encoder2_pin)) {
            direction = 1;
        } else {
            direction = -1;
        }
        interrupts();

        // Get motor position.
        // COMPLETE THIS BLOCK.
        noInterrupts();
        const uint16_t counts_per_revolution = 265;       
        position = (static_cast<int>(num_pulses)/counts_per_revolution)*3.14*2*direction;

        interrupts();
    }
    

public:
    const uint8_t encoder1_pin;
    const uint8_t encoder2_pin;
    int8_t direction;
    float position;
    uint32_t prev_time;
    bool read = false;
};

// Not inside Encoder because only free functions are interruptable.
void setEncoder(mtrn3100::Encoder& encoder) { encoder.read = true; }

}  // namespace mtrn3100
