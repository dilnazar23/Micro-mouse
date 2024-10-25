#include <Arduino.h>

#include "Encoder.hpp"
#include "Motor.hpp"
#include "UltrasonicSensor.hpp"
#include "MAFUltrasonicSensor.hpp"
#include <IoAbstractionWire.h>
#include <Wire.h>
#include <VL6180X.h>

#include "Graph.hpp"
#include "graph2ascii.hpp"

// Set up motors.
mtrn3100::Motor r_motor(2, 4, 3);
mtrn3100::Motor l_motor(7, 5, 6);

// Set up encoders.
void readLeftEncoder();
void readRightEncoder();
mtrn3100::Encoder l_encoder(18, 22, readLeftEncoder);
mtrn3100::Encoder r_encoder(19, 23, readRightEncoder);
void readLeftEncoder() { l_encoder.readEncoder(); }
void readRightEncoder() { r_encoder.readEncoder(); }

#define RANGE 1
#define address0 0x20
#define address1 0x22
int enablePin0 = 11;
int enablePin1 = 10;
VL6180X sensor0;
VL6180X sensor1;
// Set up Sensor
mtrn3100::UltrasonicSensor sensor(38, 40);
mtrn3100::MAFUltrasonicSensor<60> maf_sensor(sensor);

auto withinThreshold = [](float const distance) { return distance < 150; };

// Drive parameters.
constexpr uint8_t pwm = 100;
constexpr uint16_t forwardDuration = 500;
constexpr uint16_t turnDuration = 300;
float x0 = 0;
float y0 = 0;
float h0 = 0;

// Drive plan.
constexpr size_t drivePlanLength = 13;
char drivePlan[drivePlanLength + 1] = "LFFRFRFFLFLFF";
size_t drivePlanIndex = 0;

enum CardinalDirections {
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3,
};

// Robot pose using maze cell coordinates and heading.
int row = 0;
int col = 0;
int head = SOUTH;

// COMPLETE THIS FUNCTION.
// Drive forward then update the robot maze cell pose.
void forward() {
    l_motor.setPWM(pwm);
    r_motor.setPWM(pwm);
    delay(forwardDuration);
    l_motor.setPWM(0);
    r_motor.setPWM(0);
    delay(500);
}

// Turn left then update the robot maze cell pose.
// COMPLETE THIS FUNCTION.
void turnLeft() {
    l_motor.setPWM(-pwm);
    r_motor.setPWM(pwm);
    delay(turnDuration);
    l_motor.setPWM(0);
    r_motor.setPWM(0);
    delay(500);
}

// COMPLETE THIS FUNCTION.
// Turn right then update the robot maze cell pose.
void turnRight() {
    l_motor.setPWM(pwm);
    r_motor.setPWM(-pwm);
    delay(turnDuration);
    l_motor.setPWM(0);
    r_motor.setPWM(0);
    delay(500);
}
float x = x0;
float y = y0;
float h = h0;
// Last wheel positions.
float lastLPos = 0;
float lastRPos = 0;

void update() {
    const float leftValue = l_encoder.position;
    const float rightValue = r_encoder.position;
//    const float read_heading = imu.read();
      
     //TODO: Print encoder odometry results.
     //Serial3.println(imu.yaw());
            
    
    // COMPLETE THIS BLOCK.
    const float tL = leftValue-lastLPos;  // Change in left wheel position.
    const float tR = rightValue-lastRPos;  // Change in right wheel position.
    const float delta_h = (R/(2*L))*(tR-tL);
    const float delta_s = (R/2)*(tR+tL);
    
    
    //Serial3.println(h);
    x = x + delta_s* cos(h);
    y = y + delta_s* sin(h);
    h = h + delta_h;
    // Serial3.print("[");
    // Serial3.print(x);
    // Serial3.print(",");
    // Serial3.print(y);
    // Serial3.print(",");
    // Serial3.print(h);
    // Serial3.println("]");
    // delay(1000);
    lastLPos = leftValue;
    lastRPos = rightValue;

}
void CellPosShow(){  
  // int row = 0;
  // int col = 0;
  update();
  while (x < -125){
    Serial3.println("Moving in wrong direction");
    delay(3000);
    }
  if (y <125 && y > -125){
    row = 0;}
  else{
    row = 1;}
  if (x >= 0 && x < 125){col = 0;}
  else if(x > 125 && x <375){col = 1;}
  else {col = 2;}
  
  //char* displayHead = calculateHeading(h); 
  // Serial3.print("[");
  // Serial3.print(row);
  // Serial3.print(",");
  // Serial3.print(col);
  // Serial3.print(",");
    if (h < PI/4 && h >= -PI/4) {
    Serial3.print("E");
  } else if (h < 3*PI/4 && h >= PI/4) {
    Serial3.print("S");
  } else if (h < PI && h > 3*PI/4 || h < -3*PI/4 && h > -PI) {
    Serial3.print("W");
  } else if (h < -PI/4 && h >= -3*PI/4) {
    Serial3.print("N");
  }
  
  //calculateHeading(h);
  //Serial3.print(displayHead);
  Serial3.println("]");    
  }
// Initialise maze with nodes.
const int numRows = 3;
const int numCols = 3;
mtrn3100::Graph<int, bool> maze(1, 2, 3, 4, 5, 6, 7, 8, 9);

// Helper function to convert node positions to node indexes.
// Returns -1 if the position is invalid.
auto pos2index = [](int const row, int const col) -> int {
    // Verify row/col is valid.
    if ((row < 0 || row >= numRows) || (col < 0 || col >= numCols)) {
        return -1;
    }
    return row * numCols + col + 1;
};

// Helper function to insert edges into maze.
// Graph is a directed graph so need to do both directions.
void insert_edge(int cell1, int cell2) {
    maze.insert_edge(cell1, cell2, true);
    maze.insert_edge(cell2, cell1, true);
}

void setup() {
    // COMPLETE THIS SETUP.
    // You may need to do some more setup depending on hardware your team has selected.

    Serial.begin(115200);
    delay(100);
}

void loop() {
    // COMPLETE THIS LOOP.
    // This loop has been partially completed for you and uses the basic drive_by_command sketch.
    // Follow the comments which helps to keep the structure of the control loop.

    // Get next motion.
    switch (drivePlan[drivePlanIndex]) {
        case 'F':
            forward();
            break;
        case 'L':
            turnLeft();
            break;
        case 'R':
            turnRight();
            break;
    }

    // Debugging prints.
    Serial.print("Moving ");
    Serial.println(drivePlan[drivePlanIndex]);

    delay(100);
    //
    // Capture wall information. if no wall change it to true
    const bool frontConnected = false;
    const bool leftConnected = false;
    const bool rightConnected = false;

    // Check the heading which affects where front, left, and right cells are.
    // Get the current cell in row and col here using odometry
    
    // Cell being -1 means no cell is connected.
    const int currentCell = pos2index(row, col);
    // change the cell index here 
    int frontCell = -1;
    int leftCell = -1;
    int rightCell = -1;

    // Connect cells if there is no wall by inserting edges.
    //using the previous detect wall here
    if (true) {
    }

    // Debugging prints.
    Serial.print("F:");
    Serial.print(frontConnected);
    Serial.print(", L:");
    Serial.print(leftConnected);
    Serial.print(", R:");
    Serial.println(rightConnected);
    Serial.println(mtrn3100::graph2ascii(maze));

    // Prints edges out.
    // for (auto edge : maze.edges()) {
    //     Serial.print(mtrn3100::get<0>(edge.value));
    //     Serial.print(" <-> ");
    //     Serial.println(mtrn3100::get<1>(edge.value));
    // }

    drivePlanIndex++;

    // Add delay so we know when robot has stopped.
    delay(100);

    // Stop program if drive plan is complete.
    if (drivePlanIndex == drivePlanLength) {
        while (1) {
        }
    }
}