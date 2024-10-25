
#include <Arduino.h>

void setup() {
  Serial.begin(9600);
  Serial3.begin(9600);
}


void loop() {
  if (Serial3.available()) {
    // Read the incoming data from Bluetooth
    String incomingData = Serial3.readStringUntil('/n');

    // Print the received data
    Serial3.println("Received: " + incomingData);
  }
}