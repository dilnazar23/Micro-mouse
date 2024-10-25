#include <Arduino.h>
#include <Wire.h>
#include <VL6180X.h>
#include "Lidar.hpp"

#define address0 0x20
#define address1 0x22

int enablePin0 = 11;
int enablePin1 = 10;

VL6180X lidar0;
VL6180X lidar1;
 
THE_ENGINEERS::Lidar lidar_R(enablePin0,address0);
THE_ENGINEERS::Lidar lidar_L(enablePin1,address1);

void setup() {
    Serial.begin(9600);
    Wire.begin();
}

void loop() {
  Serial.print(lidar_R.lidar_read());
  Serial.print(",");
  Serial.println(lidar_L.lidar_read());
  
}
