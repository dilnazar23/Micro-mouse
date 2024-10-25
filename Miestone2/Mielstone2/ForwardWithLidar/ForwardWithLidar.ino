#include <Arduino.h>
#include <Wire.h>
#include "pid_controller.hpp"
#include "Encoder.hpp"
#include "Motor.hpp"
#include "Tuple.hpp"
#include <VL6180X.h>

// Set up motors.
mtrn3100::Motor l_motor(2, 4, 3);
mtrn3100::Motor r_motor(7, 5, 6);

// Set up encoders.
void readLeftEncoder();
void readRightEncoder();
mtrn3100::Encoder l_encoder(18, 22, readLeftEncoder);
mtrn3100::Encoder r_encoder(19, 23, readRightEncoder);
void readLeftEncoder() { l_encoder.readEncoder(); }
void readRightEncoder() { r_encoder.readEncoder(); }

// Set up PID
mtrn3100::PIDController pid(1, 0, 0.01, 0);
// set all to zero, move kp up 
// p - im not where i want to be 
// i - im havent been where i want to be in a while get there faster
// d - im almost where i want to be slow down 

//PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
// Drive parameters.
#define RANGE 1
const int address0=0x20;
const int address1=0x22;
int enablePin0 = 11;
int enablePin1 = 10;
VL6180X sensor0;
VL6180X sensor1;

float pwm = 100;

// Rename tuple to more readable datatype name.
using MazeCellPose = mtrn3100::Tuple<float, float, float>;

// Determines if the movement between two poses is purely translational or rotational. Returns true for translational.
bool isTranslationalMovement(MazeCellPose const& from, MazeCellPose const& to) {
    return mtrn3100::get<2>(from) == mtrn3100::get<2>(to);
}

// Pose sequence plan Esha turning 5 times .
constexpr size_t poseSequenceLength = 9;
size_t poseIndex = 0;
const MazeCellPose poseSequence[poseSequenceLength] = {{0, 0, 0}, {250, 0, 0}, {500, 0, 0}, {750, 0, 0}, {1000, 0, 0}, {1250, 0, 0}, {1500, 0, 0}, {1750, 0, 0}, {2000, 0, 0}};                                                      

//Milestone 2 is this except instead of using poseSequence want to use DriveByPlan
// {{0, 0, 0},  -->    {100, 0, 0} -- forward,  
// {100, 0, 0} --> {100, 0, M_PI / 2} -- left
// {100, 0, M_PI / 2} --> {100, 0, M_PI} -- left 
// {100, 0, M_PI} --> {0, 0, M_PI} -- forward
// {0, 0, M_PI} -->{0, 0, M_PI / 2}} -- right?


namespace inverse_kinematics {

// Current pose.
float x0 = 0;  // mm.
float y0 = 0;  // mm.
float h0 = 0;  // rad.

// Robot parameters.
float R = 23;   // mm.
float L = 130;  // mm.

// Compute left and right wheel position changes for pure translational movement.
mtrn3100::Tuple<float, float> linear(float x1, float y1) {
    // COMPLETE THIS FUNCTION.
    float thetaL = static_cast<float>(sqrt(sq(x1-x0)+sq(y1-y0)))/R ;
    float thetaR = thetaL ;
    mtrn3100::Tuple<float,float> wheel_position = {thetaL,thetaR};
    return wheel_position;
}

// Compute left and right wheel position changes for pure rotational movement.
mtrn3100::Tuple<float, float> rotational(float h1) {
    // COMPLETE THIS FUNCTION.
    float thetaL = abs((h1-h0)*(L/2)/R) ;
    float thetaR = -thetaL ;
    mtrn3100::Tuple<float,float> wheel_position = {thetaL,thetaR};
    return wheel_position;
}

// Updates the current pose with the next pose.
void update(float x1, float y1, float h1) {
    x0 = x1;
    y0 = y1;
    h0 = h1;
}

}  // namespace inverse_kinematics


template <typename T>
int sign(T val) {
    return (T(0) < val) - (val < T(0));
}

void setup() {
    Serial.begin(9600);
    Wire.begin();
    Serial3.begin(9600);

    l_motor.setPWM(0);
    r_motor.setPWM(0);
    delay(200);
    l_encoder.position = 0;
    l_encoder.position = 0;
    // Set up Lidar
    pinMode(enablePin0,OUTPUT);
    pinMode(enablePin1,OUTPUT);
    digitalWrite(enablePin0, LOW);
    digitalWrite(enablePin1, LOW); 
    delay(100); 
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
    Serial.println("Sensors ready! Start reading sensors in 3 seconds ...!");
    delay(500);
}

void loop() {
    const float thetaL = l_encoder.position;
    const float thetaR = r_encoder.position;
    // Determine is movement is translational or rotational.
    const bool isTranslation = isTranslationalMovement(poseSequence[poseIndex], poseSequence[poseIndex + 1]);
    const auto nextPose = poseSequence[++poseIndex];
    const float x1 = mtrn3100::get<0>(nextPose);
    const float y1 = mtrn3100::get<1>(nextPose);
    const float h1 = mtrn3100::get<2>(nextPose);
    mtrn3100::Tuple<float,float> wheel_position (0,0);
    // Get wheel position changes.
    if (isTranslation) {
      wheel_position= inverse_kinematics::linear(x1,y1);
    }
    else{
     wheel_position= inverse_kinematics::rotational(h1);
    };

    float l_expect = mtrn3100::get<0>(wheel_position);
    float r_expect = mtrn3100::get<1>(wheel_position);
       
    //Debugging
     float speed = 0.1;
     //float speed = 10.0001;
    Serial.println(speed);
    l_motor.setPWM(speed);
    //r_motor.setPWM(speed);
    while (1){};

    float error_l = l_expect - (l_encoder.position-thetaL);
    float error_r = r_expect - (r_encoder.position-thetaR);
    float errors[2] = {error_l,error_r};
    // float l_lidar = sensor0.readRangeContinuousMillimeters();
    // float r_lidar = sensor1.readRangeContinuousMillimeters();
    float  h_error_lidar = 0;
    //Debug printing
    Serial.print("initial left motor error: ");
    //end Esha
    Serial.print(error_l); //should be decreasing 
    Serial.print("  ");
    //Esha 
    Serial.print("initial right motor error: ");
    //end Esha
    Serial.print(error_r);
    Serial.print("  ");
    Serial.print("initial right motor error: ");
    Serial.println(h_error_lidar);
    //while(1){};

    while((fabs(error_l) >=0.1) || (fabs(error_r) >=0.1)){
      //update encoder error
      error_l = l_expect - (l_encoder.position-thetaL);
      error_r = r_expect - (r_encoder.position-thetaR);
      errors[0] = error_l;
      errors[1] = error_r;
      //update lidar heading error
      const float l_lidar = sensor0.readRangeContinuousMillimeters();
      const float r_lidar = sensor1.readRangeContinuousMillimeters();
      //ger lidar signal:: if there is something on the right(reading<70)
      //signal =0 for right,signal = 1 for left    
      int wall_detect = 1000;
      if (l_lidar<65){
        wall_detect = 0;
        h_error_lidar = 0.01*fabs(l_lidar-75);
        }
      else if (r_lidar<75){
        wall_detect =1;
        // lidar_error_right = r_lidar - 75, pass it to the pid controller
        h_error_lidar = 0.01*fabs(r_lidar-75);
        } 
      else{h_error_lidar = 0;}   
    
      // lidar_error_right = r_lidar - 75, pass it to the pid controller
      //h_error_lidar = 0.05*fabs(l_lidar - r_lidar);
      
      float* pid_result = pid.compute(errors,h_error_lidar, wall_detect);
      float l_pid_speed = pid_result[0];
      float r_pid_speed = pid_result[1];
      
      if (fabs(error_l) >=0.1) l_motor.setPWM(1.5*l_pid_speed);
      else l_motor.setPWM(0);
      if (fabs(error_r) >=0.1) r_motor.setPWM(r_pid_speed);
      else r_motor.setPWM(0);
      //printing for see the error change
      Serial.print("l_pid speed: ");
      Serial.println(l_pid_speed);
      Serial.print("r_pid speed: ");
      Serial.println(r_pid_speed);
      Serial.print("l_error: ");
      Serial.println(error_l);
      Serial.print("r_error: ");
      Serial.println(error_r);
    }
    
    // Set motors to 0 when wheel position has been reached.
    l_motor.setPWM(0);
    r_motor.setPWM(0);

    Serial.println("one command done");
    delay(10);

    // const float cur_motor_pos = 0;
    // r_encoder.position = 0;
    
    inverse_kinematics::update(x1,y1,h1);
    // So we can see when a movement has ended.
    delay(1000);
    

    // Stop program if drive plan is complete.
    while (poseIndex == poseSequenceLength - 1) {      
    }
}
