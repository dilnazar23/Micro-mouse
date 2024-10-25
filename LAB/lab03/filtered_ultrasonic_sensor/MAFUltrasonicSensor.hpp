#pragma once

#include "MovingAverageFilter.hpp"
#include "UltrasonicSensor.hpp"
#include "MovingAverageFilter.hpp"
namespace mtrn3100 {

// COMPLETE THIS CLASS.
template <size_t N>
class MAFUltrasonicSensor : public UltrasonicSensor {
public:
    MAFUltrasonicSensor(UltrasonicSensor& sensor) : UltrasonicSensor(sensor) {
      }
    UltrasonicSensor& Sensor;
    // The sensor is ready to be used if the filter is full.
    bool isReady() const { 
      return filter.isFull();
      }

    // Take a sample from the sensor and put it into the filter.
    void sample() {
      //delay(10);
      //data= Sensor.echo();      
      filter.sample(echo()); 
      }

    // Returns the average of the filter.
    float value() const { return filter.average(); }

    // Returns the capacity of the filter.
    size_t capacity() const { return filter.size(); }

private:
    //float data;
    MovingAverageFilter<float, N> filter;
    //UltrasonicSensor& Sensor;
//public:
//    float data;
};

}  // namespace mtrn3100
