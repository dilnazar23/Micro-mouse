#include <Arduino.h>

#include "MAFUltrasonicSensor.hpp"
#include "UltrasonicSensor.hpp"

mtrn3100::UltrasonicSensor sensor(40, 38);
mtrn3100::MAFUltrasonicSensor<60> maf_sensor(sensor);

void setup() { Serial.begin(9600); 
}

void loop() {
    //delay(100);
    maf_sensor.sample();
    float filtered_dis = maf_sensor.value();
    if (maf_sensor.isReady()) {
        //delay(100);
        Serial.print("Ultrasonic:");
        Serial.print(sensor.echo());
        Serial.print(",");
        Serial.println(filtered_dis);
    }
}
