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
#include "Graph.hpp"

//DECLARE GLOBAL ROBOT PARAMETER
const float L=130,R=23;
float x,y,h,x1,y1,h1,h0;
//DEFINE PARKING GLOBAL VARS
#define PARKING_SPEED 40
#define PARKING_ADJUST_SPEED 2
int PARKING_DIS=80;
//DEFINE THRESHOLD VALUE FOR DIFFIRENT SENSORS
#define Ultra_thres 60
#define Lidar_thres 60
//DEFINE WALLS AS GLOBAL VARIABLE
bool WallRight=false, WallLeft=false,WallFront=false;
using MazeCellPose = mtrn3100::Tuple<float, float, float>;
constexpr size_t poseSequenceLength = 7;
size_t poseIndex = 0;
const MazeCellPose poseSequence[poseSequenceLength] = {{0, 0, 0},{236, 0, 0},{472,0,0},{472,0,M_PI/2},{472,236,M_PI/2},{472,236,M_PI},{236,236,M_PI},{0,236,M_PI},{0,236,M_PI} };
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

// Set up Sensor
// mtrn3100::UltrasonicSensor sensor(40, 38);
// mtrn3100::MAFUltrasonicSensor<10> maf_sensor(sensor);


void insert_edge(mtrn3100::Graph<int, int>& maze, int cell1, int cell2) {
    // Graph is a directed graph so need to do both directions.
    maze.insert_edge(cell1, cell2, 0);
    maze.insert_edge(cell2, cell1, 0);
}
    int p11{1}, p12{2}, p13{3}, p14{4}, p15{5}, p16{6}, p17{7}, p18{8}, p19{9};
    int p21{10}, p22{11}, p23{12}, p24{13}, p25{14}, p26{15}, p27{16}, p28{17}, p29{18};
    int p31{19}, p32{20}, p33{21}, p34{22}, p35{23}, p36{24}, p37{25}, p38{26}, p39{27};
    int p41{28}, p42{29}, p43{30}, p44{31}, p45{32}, p46{33}, p47{34}, p48{35}, p49{36};
    int p51{37}, p52{38}, p53{39}, p54{40}, p55{41}, p56{42}, p57{43}, p58{44}, p59{45};

    mtrn3100::Graph<int, int> map(p11, p12, p13, p14, p15, p16, p17, p18, p19, p21, p22, p23, p24, p25, p26, p27,
                                       p28, p29, p31, p32, p33, p34, p35, p36, p37, p38, p39, p41, p42, p43, p44, p45,
                                       p46, p47, p48, p49, p51, p52, p53, p54, p55, p56, p57, p58, p59);
  // record my cell number and my heading,if heading = 0/pi,see front cell+col, else,

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
// RUN WITH PARKING
//--- CHECK WALL
//--- IF L/R--> PARKING WITH LIDAR
//--- IF NO WALL--> PARKING WITH IMU
// PARKING WITH LIDAR
//---GET LEFT-RIGHT > 0 --> CAL PID VALUE --> FEED TO MOTOR
//--- REPEAT UNTILL LEFT-RIGHT IS VERY SMALL
// PARKING WITH IMU
//--- ODOMETRY WITH IMU
//--- UPDATE GOAL BY NEW POS
//--- RUN WITH LOW SPEED


// FORWARD
// void Forward(){
//   // check how much distance i should travel
//   // if i travel more than 250, i do forward+parking
//   float d = sqrt(sq(x1-x)+sq(y1-y));
//   if (d <250){forwardEncoder();}
//   else{Parking();}
// }


// UPDATE TO GET CURRENT POS
  // RUN IN BANGBANG IN LOW THRESHOLD SPEED
  // DETECT WALL
  // INCREASE OR DECREASE CURRENT SPEED 
  // RUN UNTILL CURRENT POS == GOAL
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
    // Serial.print(x);
    // Serial.print(",");
    // Serial.print(y);
    // Serial.print(",");
    // Serial.print(h);
    // Serial.println(); 
  }
  imu.getYaw();
  l_motor.setPWM(0);
  r_motor.setPWM(0);
  imu.getYaw();
  delay(30);
  imu.getYaw();
  ODOMETRY::posUpdate();
  Serial.print(F("Parking Finished and this is my pose: "));
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(",");
  Serial.print(h);
  Serial.println();  
}
void setup() { 
 Wire.begin();
  Serial.begin(9600);
  Serial3.begin(9600); 

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
  // // set up Ultra sensor
  // Serial.println(F("Initializing Ultrasonic Sensor..."));
  // maf_sensor.initialize();
  // Serial.println("Ultrasonic Sensor ready! Start reading sensors in 3 seconds ...!");
  // delay(3000);
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
  // Serial.print(F("x:"));
  // Serial.print(x);
  // Serial.print(F("y:"));
  // Serial.print(y);
  // Serial.print(F("h:"));
  // Serial.println(h);
  imu.getYaw();
  inverse_kinematics::tarPosUpdate();  
  // while(poseIndex == 5){}
// Stop program if drive plan is complete.
  while (poseIndex == poseSequenceLength - 1) {}
  
}
