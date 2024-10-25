#pragma once

#include <Arduino.h>

namespace mtrn3100 {

class UltrasonicSensor {
public:
    // COMPLETE THIS CONSTRUCTOR.
    UltrasonicSensor(uint8_t trigger_pin, uint8_t echo_pin): trigger_pin{trigger_pin}, echo_pin{echo_pin} 
    {
      pinMode(trigger_pin,OUTPUT); //OUTPUT or INPUT??
      pinMode(echo_pin,INPUT);
//      // Clears the trigPin
//      digitalWrite(trigger_pin, LOW);
//      delayMicroseconds(2);
//      // Sets the trigPin on HIGH state for 10 micro seconds
//      digitalWrite(trigger_pin, HIGH);
//      delayMicroseconds(10);
//      digitalWrite(trigger_pin, LOW);
      }

    // COMPLETE THIS FUNCTION.
    float echo() const { 
       // Clears the trigPin
      digitalWrite(trigger_pin, LOW);
      delayMicroseconds(2);
      // Sets the trigPin on HIGH state for 10 micro seconds
      digitalWrite(trigger_pin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigger_pin, LOW);
      //digitalWrite(trigger_pin,HIGH);
      long duration= pulseIn(echo_pin,HIGH);
      float dis= duration * 0.343 / 2;
      //delay(500);
      return dis;
      }

private:
    const uint8_t trigger_pin;
    const uint8_t echo_pin;
};

}  // namespace mtrn3100
