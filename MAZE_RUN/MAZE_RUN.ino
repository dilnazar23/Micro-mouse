#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <VL6180X.h>
#include "Encoder.hpp"
#include "Motor.hpp"
#include "MPU6050.hpp"
#include "MAFUltrasonicSensor.hpp"
#include "UltrasonicSensor.hpp"
#include "pid_controller.hpp"
#include "Tuple.hpp"

//DECLARE GLOBAL ROBOT PARAMETER
const float L=130,R=23;
float x,y,h,x1,y1,h1,h0;
//DEFINE PARKING GLOBAL VARS
#define PARKING_SPEED 35
#define PARKING_ADJUST_SPEED 2
int PARKING_DIS=80;
//DEFINE THRESHOLD VALUE FOR DIFFIRENT SENSORS
#define Ultra_thres 60
#define Lidar_thres 60
//DEFINE WALLS AS GLOBAL VARIABLE
bool WallRight=false, WallLeft=false,WallFront=false;

// Set up PID
mtrn3100::PIDController pid(8, 0, 0, 0);

// Set up motors.
mtrn3100::Motor r_motor(2, 4, 3);
mtrn3100::Motor l_motor(7, 5, 6);

// Set up encoders.
void readLeftEncoder();
void readRightEncoder();
mtrn3100::Encoder r_encoder(18, 22, readRightEncoder);
mtrn3100::Encoder l_encoder(19, 23, readLeftEncoder);
void readLeftEncoder() { l_encoder.readEncoder(); }
void readRightEncoder() { r_encoder.readEncoder(); }

// Set up Lidar
#define RANGE 1
#define address0 0x20
#define address1 0x22
int enablePin0 = 11;
int enablePin1 = 10;
VL6180X sensor0;
VL6180X sensor1;

// Set up IMU
IMU imu;
namespace ASCII2POSE{
  float startHead = 0;
  void generate_and_print_map() {
  auto const actual = mtrn3100::ascii2graph(maze);
  for (auto e : actual.edges()) {
      int src = mtrn3100::get<0>(e.value);
      int dst = mtrn3100::get<1>(e.value);
      Serial.print("{");
      Serial.print(src);
      Serial.print(", ");
      Serial.print(dst);
      Serial.println("},");

  }
}


template<typename N, typename E>
mtrn3100::LinkedList<N> bfs_single(mtrn3100::Graph<N, E> const& g, N const& src, N const& dst) {
  mtrn3100::LinkedList<mtrn3100::LinkedList<N>> queue;
  queue.push_front({ src });

  mtrn3100::LinkedList<N> visited;
  mtrn3100::LinkedList<N> path;

  while (!queue.empty()) {
    mtrn3100::LinkedList<N> currentPath = queue.front();
    queue.pop_front();

    //Getting the current node in the path
    N currentNode = currentPath.back();

    if (currentNode == dst) {
      path = currentPath;
      break;
    }

    // If current node has been visted it skips the for loop
    if (visited.contains(currentNode))
      continue;

    // Adds it the visited path
    visited.push_back(currentNode);

    mtrn3100::LinkedList<N> nodes = g.nodes(currentNode);


    for (auto it = nodes.begin(); it != nodes.end(); ++it) {
      N node = it->value;
      mtrn3100::LinkedList<N> newPath = currentPath;
      newPath.push_back(node);
      queue.push_back(newPath);
    }
  }
  return path;
}


//printing the shortest paths
template<typename N>
void printShortestPaths(const mtrn3100::LinkedList<mtrn3100::LinkedList<N>>& shortestPaths) {
  // Serial3.println(F("Shortest Paths:"));
  for (size_t i = 0; i < shortestPaths.size(); i++) {
    // Serial3.print("Path " + String(i + 1) + ": ");
    for (size_t j = 0; j < shortestPaths[i].size(); j++) {
      Serial3.print(shortestPaths[i][j]);
      if (j < shortestPaths[i].size() - 1) {
        Serial3.print(F(", "));  // change this to an array output
      }
    }
    Serial3.println();
  }
};


template<typename N, typename E>
void addNodesAndEdges(mtrn3100::Graph<N, E>& graph, const int edges[][2], size_t edgeCount) {
  for (size_t i = 0; i < edgeCount; i++) {
    N source = edges[i][0];
    N destination = edges[i][1];
    graph.insert_node(source);
    graph.insert_node(destination);
    graph.insert_edge(source, destination, 1);
  }
};

mtrn3100::Tuple<float, float, float>* yes;

int* maze_run(int path[], int size, float start_head) {
  mtrn3100::Tuple<float, float, float> coordinates[100];

  //initialise all 'prev' variables to match the starting pose
  float prev_head = start_head;
  auto firstrc = nodetorowcol(path[0]);
  int prev_row = mtrn3100::get<0>(firstrc);
  int prev_col = mtrn3100::get<1>(firstrc);
  int prev_node = path[0];
  float prev_x = 250 * (prev_row) + 125;
  float prev_y = 250 * (prev_col) + 125;

  int counter = 0;       //iterates through pose array
  int path_counter = 0;  //iterates through path array
  while (true) {
    int curr_node = path[path_counter];
    auto rc = nodetorowcol(curr_node);
    int curr_row = mtrn3100::get<0>(rc);
    int curr_col = mtrn3100::get<1>(rc);
    float x_pose = prev_x;
    float y_pose = prev_y;
    float curr_head = prev_head;
    float next_head = nodetodirection(prev_node, curr_node, prev_head);

    if (abs(next_head - curr_head) < 0.01) {  //If the current heading matches the direction to be travelled, move cell
      x_pose = 250 * curr_col + 125;
      y_pose = 250 * curr_row + 125;
      coordinates[counter] = { x_pose, y_pose, curr_head };
      counter++;
      path_counter++;


    } else if (abs(abs(next_head - curr_head) - M_PI) < 0.01) {  //If the current heading is directly opposite the direction to be travelled, turn right twice
      curr_node = path[path_counter - 1];
      curr_head = turnRight(prev_head);
      coordinates[counter] = { x_pose, y_pose, curr_head };
      counter++;

      prev_head = curr_head;
      curr_head = turnRight(prev_head);
      coordinates[counter] = { x_pose, y_pose, curr_head };
      counter++;
    } else if (abs(abs(next_head - abs(curr_head)) - M_PI / 2) < 0.01) {  //If the current heading requires one turn to match the direction to be travelled, determine direction to turn
      curr_node = path[path_counter - 1];
      if (abs(next_head) - abs(curr_head) < 0.0 || (next_head - M_PI < 0.01 && curr_head + M_PI / 2 < 0.01)) {  //Turn left
        curr_head = turnLeft(prev_head);
        coordinates[counter] = { x_pose, y_pose, curr_head };
        counter++;

      } else if (abs(next_head) - abs(curr_head) > 0.0) {  //Turn right
        curr_head = turnRight(prev_head);
        coordinates[counter] = { x_pose, y_pose, curr_head };
        counter++;
      }
    }

    prev_row = curr_row;
    prev_col = curr_col;
    prev_x = x_pose;
    prev_y = y_pose;
    prev_head = curr_head;
    prev_node = curr_node;

    if (path_counter == size) {
      break;
    }
  }


  int new_size = 0;
  int new_counter = 0;
  while (true) {
    if (mtrn3100::get<0>(coordinates[new_counter]) == 0.0) {
      break;
    }
    new_counter++;
  }
  new_size = new_counter;
  mtrn3100::Tuple<float, float, float> new_coordinates[new_size];

  for (int i = 0; i < new_size; i++) {
    new_coordinates[i] = coordinates[i];
  }

  yes = new_coordinates;
  return sizeof(new_coordinates) / sizeof(new_coordinates[0]);
};
 void ascii2pose(){
  int src = SOURCE_CELL;        // starting position
  int dst = DST_CELL;  // destination position
  // int ascii_size=sizeof(maze);
  // while(1){Serial.println(ascii_size);}
  auto const actual = mtrn3100::ascii2graph(maze);
  mtrn3100::LinkedList<mtrn3100::LinkedList<int>> shortestPaths = bfs_single(actual, src, dst);
  printShortestPaths(shortestPaths);
  // Extract the path from shortestPaths
  mtrn3100::LinkedList<int> shortestPath = shortestPaths[0];  // Assuming you want the first shortest path
                                                              // Convert the path to a regular C++ array
  int pathArray[shortestPath.size()];
  int index = 0;
  for (auto it = shortestPath.begin(); it != shortestPath.end(); ++it) {
    pathArray[index++] = it->value;
  }

  // Call maze_run function with the path array
  // float startHead = 0; // You might need to set the start heading
  int mazeSize = maze_run(pathArray, shortestPath.size(), startHead);

  // Print the coordinates generated by maze_run
  for (int i = 0; i < mazeSize; i++) {
    Serial.print("{");
    Serial.print(mtrn3100::get<0>(poseSequence[i]));
    Serial.print(", ");
    Serial.print(mtrn3100::get<1>(poseSequence[i]));
    Serial.print(", ");
    Serial.print(mtrn3100::get<2>(poseSequence[i]));
    Serial.println("}");
  }
} 
}//namespace ASCII2POSE

namespace HARDWARE{
void detectWall() {
  //const int front = maf_sensor.value();
  const int right = sensor0.readRangeContinuousMillimeters();
  const int left = sensor1.readRangeContinuousMillimeters(); 
  int wall_min = min(min(left, right),Lidar_thres);
  if (wall_min==left){
    WallLeft=true;
    WallRight=false;
    Serial.println("left wall");
  }
  else if (wall_min==right){
    WallRight=true;
    WallLeft=false;
    Serial.println("right wall");
  }
  //else if (wall_min==front){{Serial.println("left wall");return "F";}
  else{
    WallRight=false;
    WallLeft=false;}    
}
void HardwareIni(){
  //----HARDWARE INITIALISE----  
  //Set up motor
  r_motor.setPWM(0);
  l_motor.setPWM(0);
  delay(100);
  //Set up Encoder
  r_encoder.position = 0;
  l_encoder.position = 0;
  //Set up IMU
  // Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
  //setup lidar
  pinMode(enablePin0,OUTPUT);
  pinMode(enablePin1,OUTPUT);
  digitalWrite(enablePin0, LOW);
  digitalWrite(enablePin1, LOW); 
  delay(500); 
    // Sensor0
  Serial.println("Start Sensor 0");
  digitalWrite(enablePin0, HIGH);
  delay(50);
  sensor0.init();
  sensor0.configureDefault();
  sensor0.setAddress(address0);
  Serial.println(sensor0.readReg(0x212),HEX); // read I2C address
  sensor0.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  sensor0.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  sensor0.setTimeout(500);
  sensor0.stopContinuous();
  sensor0.setScaling(RANGE); // configure range or precision 1, 2 oder 3 mm
  delay(300);
  sensor0.startInterleavedContinuous(100);
  delay(100);

    // Sensor1
  Serial.println("Start Sensor 1");
  digitalWrite(enablePin1, HIGH);
  delay(50);
  sensor1.init();
  sensor1.configureDefault();
  sensor1.setAddress(address1);
  Serial.println(sensor1.readReg(0x212),HEX);
  sensor1.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
  sensor1.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
  sensor1.setTimeout(500);
  sensor1.stopContinuous();
  sensor1.setScaling(RANGE);
  delay(300);
  sensor1.startInterleavedContinuous(100);
  delay(100);
  Serial.println("Lidars ready! Start reading sensors in 3 seconds ...!");
  delay(1000);
}
}//namespace hardware

namespace ODOMETRY{

float lastLPos,lastRPos;

void pos_init(){ //need to modofy as read the initial pose from bluetooth or pose sequence
  //  READ THE FIRST FIRST CELL AND CONVERT TO INITIAL POS  
  const auto inPose = poseSequence[0];
  x = mtrn3100::get<0>(inPose);
  y = mtrn3100::get<1>(inPose);
  h0 = mtrn3100::get<2>(inPose);
  h=h0;
// Last wheel positions.
  lastLPos = 0;
  lastRPos = 0;
}
void  posUpdate(){    
    const float leftValue = l_encoder.position;
    const float rightValue = r_encoder.position;    
    float dL = R*(leftValue-lastLPos);  // distance travel by the left wheel
    float dR = R*(rightValue-lastRPos);  // distance travel by the right wheel
    float d = (dL+dR)/2; // distance travel by the center of robot
    float delta_h = (dR-dL)/L; // change of heading
    // update x,y,h base on the change
    x = x + d * cos(h+(delta_h/2));
    y = y + d * sin(h+(delta_h/2));
    //h=imu.getYaw()*0.0174;
    h = h + delta_h;
    //update the wheel position
    lastLPos = leftValue;
    lastRPos = rightValue; 
}
// CHECK IF I GET TO GOAL
//--- CHECK X Y
bool getGoal(){
  return abs(x-x1)<10 && abs(y-y1)<10;
    }

} // namespace ODOMETRY

namespace inverse_kinematics {

// Compute left and right wheel position changes for pure translational movement.
mtrn3100::Tuple<float, float> linear() {  
  float thetaL = static_cast<float>(sqrt(sq((x1-cos(h)*PARKING_DIS) - x) + sq((y1-sin(h)*PARKING_DIS) - y))) / R;
  float thetaR = thetaL;
  mtrn3100::Tuple<float, float> wheel_position = { thetaL, thetaR };
  return wheel_position;
}

// Compute left and right wheel position changes for pure rotational movement.
mtrn3100::Tuple<float, float> rotational() {
  // pid.tune()
  float thetaR = (h1 - h) * (L / 2) / R;
  float thetaL = -thetaR;
  mtrn3100::Tuple<float, float> wheel_position = { thetaL, thetaR };
  return wheel_position;
}

mtrn3100::Tuple<float, float> GetInverse(){   
  const bool isTranslation = (h0==h1);  
  mtrn3100::Tuple<float, float> wheel_position(0, 0);
  if (isTranslation){
    wheel_position =linear();
    }
  else{
  wheel_position =rotational();
  }
  return wheel_position;
}
// Updates the current pose with the next pose.
void tarPosUpdate() {
  x = x1;
  y = y1;
  h = h1;
  h0 = h1;
}

}  // namespace inverse_kinematics


void fix_turnLeft(){
  l_motor.setPWM(32);
  r_motor.setPWM(-32);
}
void fix_turnRight(){
  l_motor.setPWM(-32);
  r_motor.setPWM(32);
}
// use imu to fix head
void fixHead_imu(){  

  float heading_err = imu.getYaw()-(h0*57.32);
  while(fabs(heading_err)>10){
    heading_err = imu.getYaw()-(h0*57.32);
    Serial.print(imu.getYaw());
    Serial.print(",");
    Serial.println(h0*57.32);
    if (heading_err > 0){fix_turnRight();}
    else{fix_turnRight();}
  }
  // while((imu.getYaw()-(h0*57.32))>10){fix_turnLeft();}
  // while((imu.getYaw()-(h0*57.32))<-10){fix_turnRight();}
  // l_motor.setPWM(0);
  r_motor.setPWM(0);
  delay(50);
  ODOMETRY::posUpdate();
}
// RUN WITH ENCODER PID
void forwardEncoder(){ 
  imu.getYaw();
   const float thetaL = l_encoder.position;
  const float thetaR = r_encoder.position;
  mtrn3100::Tuple<float, float> wheel_pos(0, 0);
  wheel_pos=inverse_kinematics::GetInverse();
  float l_expect = mtrn3100::get<0>(wheel_pos);
  float r_expect = mtrn3100::get<1>(wheel_pos);  
  float error_l = l_expect - (l_encoder.position - thetaL);
  float error_r = r_expect - (r_encoder.position - thetaR);
  float errors[2] = { error_l, error_r };
  while((fabs(error_l) >=0.02) || (fabs(error_r) >=0.02)){
    imu.getYaw();
      error_l = l_expect - (l_encoder.position-thetaL);
      error_r = r_expect - (r_encoder.position-thetaR);
      errors[0] = error_l;
      errors[1] = error_r;
      float* pid_result = pid.compute(errors);
      float l_pid_speed = pid_result[0];
      float r_pid_speed = pid_result[1];
      //Serial.println(l_pid_speed);
      if (fabs(error_l) >0.02){ 
        l_motor.setPWM(l_pid_speed);      
        }
      else {
        l_motor.setPWM(0);
        //delay(10);
        pid.resetIntegral(1);
        };

      if (fabs(error_r) >=0.02) r_motor.setPWM(r_pid_speed);
      else {
        r_motor.setPWM(0);
       // delay(10);
        pid.resetIntegral(0);
        };      
      Serial.print("l_error: ");
      Serial.println(error_l);
      Serial.print("r_error: ");
      Serial.println(error_r);
      //posUpdate();
    }
    ODOMETRY::posUpdate();    
    PARKING_DIS = 80;
}

void Parking(){   
  while(1){
    imu.getYaw();
    ODOMETRY::posUpdate(); 
    if(ODOMETRY::getGoal()){break;};   
    HARDWARE::detectWall();
    if(WallLeft){      
        // Serial.println(F("I'm running with lidar parking with left wall!"));
        l_motor.setPWM(PARKING_SPEED+PARKING_ADJUST_SPEED);
        r_motor.setPWM(PARKING_SPEED-PARKING_ADJUST_SPEED);
        ODOMETRY::posUpdate();
        if(ODOMETRY::getGoal()){break;};
    }
    else if(WallRight){
        // Serial.println(F("I'm running with lidar parking with right wall!"));
        l_motor.setPWM(PARKING_SPEED-PARKING_ADJUST_SPEED);
        r_motor.setPWM(PARKING_SPEED+PARKING_ADJUST_SPEED);
        ODOMETRY::posUpdate();
        if(ODOMETRY::getGoal()){break;};
    }
    else{ 
      Serial.print("I'm trying to park with encoder");
      PARKING_DIS=0;     
      forwardEncoder();      
      ODOMETRY::posUpdate();
      break;    
      
    }
   ODOMETRY::posUpdate();
   if(ODOMETRY::getGoal()){break;};
  }
  imu.getYaw();
  l_motor.setPWM(0);
  r_motor.setPWM(0);
  imu.getYaw();
  delay(30);
  imu.getYaw();
  ODOMETRY::posUpdate();
}
void setup() { 
 Wire.begin();
  Serial.begin(9600);
  Serial3.begin(9600);
  HARDWARE::initial();
  imu.getYaw();
  ODOMETRY::pos_init(); // initialise pose
}

void loop() {
  imu.getYaw();  
  const auto nextPose = poseSequence[++poseIndex];
  x1 = mtrn3100::get<0>(nextPose);
  y1 = mtrn3100::get<1>(nextPose);
  h1 = mtrn3100::get<2>(nextPose);
  fixHead_imu();  
  forwardEncoder();
  Parking();
  l_motor.setPWM(0);
  r_motor.setPWM(0);
  imu.getYaw();
  //delay(20);
  Serial.print(F("One command finished and this is my pose: "));
  ODOMETRY::posUpdate();
  imu.getYaw();
  inverse_kinematics::tarPosUpdate();  

  while (poseIndex == poseSequenceLength - 1) {}
  
}
