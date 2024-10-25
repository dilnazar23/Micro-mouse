#pragma once

#include <Arduino.h>

namespace mtrn3100 {

class UltrasonicSensor {
public:    
    UltrasonicSensor(uint8_t trigger_pin, uint8_t echo_pin): trigger_pin{trigger_pin}, echo_pin{echo_pin} 
    {
      pinMode(trigger_pin,OUTPUT); 
      pinMode(echo_pin,INPUT);
      }
    
    float echo() const {        
      digitalWrite(trigger_pin, LOW);
      delayMicroseconds(2);
      // Sets the trigPin on HIGH state for 10 micro seconds
      digitalWrite(trigger_pin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigger_pin, LOW);      
      long duration= pulseIn(echo_pin,HIGH);
      float dis= duration * 0.343 / 2;      
      return dis;
      }

private:
    const uint8_t trigger_pin;
    const uint8_t echo_pin;
};

}  // namespace mtrn3100
