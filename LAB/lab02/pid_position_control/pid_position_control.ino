#include <Arduino.h>

#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"

// Motor pins.
mtrn3100::Motor l_motor(2, 4, 3);
mtrn3100::Motor r_motor(7, 5, 6);

// Set up PIDs.
mtrn3100::PIDController l_pos_pid(1, 0.5, 0, 60, 255, 1);
mtrn3100::PIDController r_pos_pid(1, 0.5, 0, 60, 255, 1);

// Encoder pins.
void readLeftEncoder();
void readRightEncoder();
mtrn3100::Encoder l_encoder(18, 22, readLeftEncoder);
mtrn3100::Encoder r_encoder(19, 23, readRightEncoder);
void readLeftEncoder() { l_encoder.readEncoder(); }
void readRightEncoder() { r_encoder.readEncoder(); }

void setup() { Serial.begin(115200); }

// Global variables.
const float rev = 2 * M_PI;
float target = rev;
uint8_t times_spun = 0;

// Used for recording and graphing the motor position.
constexpr uint16_t max_samples = 1000;
unsigned curr_sample_size = 0;
float positions[max_samples];
unsigned loop_count = 0;

void saveSample(float sample) {
    positions[curr_sample_size] = sample;

    noInterrupts();
    curr_sample_size++;
    interrupts();
}

void plotMotorState() {
    Serial.print("Encoder Data:");
    for (int i = 0; i < curr_sample_size; i++) {
        Serial.println(positions[i]);
    }
}

void loop() {
    // Drive the motor.
    noInterrupts();
    float actual = l_encoder.position;
    float signal = l_pos_pid.compute(target, actual);
    l_motor.setPWM(signal);
    interrupts();

    // Save samples every now and then because memory is limited.
    if (loop_count == 100) {
        saveSample(l_encoder.position);
        loop_count = 0;
    }

    // Have completed a full revolution.
    if (signal == 0) {
        // Save the motor position during the pause 100 times.
        for (unsigned i = 0; i < 100; i++) {
            saveSample(l_encoder.position);
        }

        // Just for physical effect for pause.
        delay(1000);

        noInterrupts();
        times_spun++;
        target += rev;
        interrupts();

        // Stop program if wheel has already spun twice.
        if (times_spun >= 2) {
            plotMotorState();
            while (1) {
            }
        }
    }

    loop_count++;
}
