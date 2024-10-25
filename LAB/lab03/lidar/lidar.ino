// COMPLETE THIS SKETCH.

#include <Wire.h>
#include <VL6180X.h>
const int address0=0x20;

VL6180X lidar_L;
//VL6180X lidar_R;

void setup() 
{
  //analogWrite(12,5);
  Serial.begin(9600);
  Wire.begin(lidar_L.getAddress());
  //Wire.begin(lidar_R.getAddress());
  
  lidar_L.init();
  lidar_L.configureDefault();
  lidar_L.setTimeout(500);
  
  // lidar_R.init();
  // lidar_R.configureDefault();
  // lidar_R.setTimeout(500);
}

void loop() 
{ 
//  Serial.print(lidar_L.getAddress());
//  Serial.print(",");
//  Serial.print(lidar_R.getAddress());
  
  Serial.print(lidar_L.readRangeSingleMillimeters());
  //Serial.print(",");
  //Serial.print(lidar_R.readRangeSingleMillimeters());
  if (lidar_L.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  
  Serial.println();
}
