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
      filter.sample(echo()); 
      }

    // Returns the average of the filter.
    float value() const { return filter.average(); }

    // Returns the capacity of the filter.
    size_t capacity() const { return filter.size(); }

    //initialise the maf_sensor by clear the buffer and let it full
    void initialize(){
      filter.clear();
      while (!isReady()){
        sample();
      }
    }

private:    
    MovingAverageFilter<float, N> filter;   
};

}  // namespace mtrn3100
