// MielStone1 all tasks
// THE ENGINEERS
#include <Arduino.h>
#include <IoAbstractionWire.h>
#include <LiquidCrystalIO.h>
#include <Wire.h>
#include <VL6180X.h>

//#include "Robot.hpp"
#include "Encoder.hpp"
#include "Motor.hpp"
//#include "LCD.hpp"
#include "IMU.hpp"
#include "MAFUltrasonicSensor.hpp"
#include "UltrasonicSensor.hpp"
// #include "Lidar.hpp"
//#include "Bluetooth.hpp"


// bluetooth receive command
// Move by Command
// Get position
// Show position
// Detect wall
// Show Detected wall

// Setting testing parameters

    // Drive parameters.
constexpr uint8_t pwm = 80;
constexpr uint16_t forwardDuration = 900;
constexpr uint16_t turnDuration = 230;
    // Robot parameters.
const float R = 22.5;  // mm.
const float L = 75;  // mm.
//THE_ENGINEERS::Robot robot(L,R);
const float x0 = 0;
const float y0 = 0;
const float h0 = -0.03;

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

// Set up IMU
mtrn3100::IMU imu(0);
// Set up Sensor
mtrn3100::UltrasonicSensor sensor(40, 38);
mtrn3100::MAFUltrasonicSensor<60> maf_sensor(sensor);

// Set up Lidar
#define RANGE 1
#define address0 0x20
#define address1 0x22
int enablePin0 = 11;
int enablePin1 = 10;
VL6180X sensor0;
VL6180X sensor1;
// Set up bluetooth

// Set up LCD
//LiquidCrystalI2C_RS_EN(lcd, 0x3F, false);
// Set up Robot
//THE_ENGINEERS:: Robot robot(L,R);



// move 1 cell function
void forward() {
    l_motor.setPWM(pwm);
    r_motor.setPWM(pwm);
    delay(forwardDuration);
    l_motor.setPWM(0);
    r_motor.setPWM(0);
    delay(500);
}

void turnLeft() {
    l_motor.setPWM(-pwm);
    r_motor.setPWM(pwm);
    delay(turnDuration);
    l_motor.setPWM(0);
    r_motor.setPWM(0);
    delay(500);
}

void turnRight() {
    l_motor.setPWM(pwm);
    r_motor.setPWM(-pwm);
    delay(turnDuration);
    l_motor.setPWM(0);
    r_motor.setPWM(0);
    delay(500);
}

//Read command by bluetooth function
String ReadCommand(){
    Serial3.println("Enter command:");
    while(Serial3.read() == -1){};        
    if (Serial.available() > 0 ) {
        Serial3.write(Serial.read());
        Serial3.print("Read it");
    }
    if (Serial3.available()) {
        Serial.write(Serial3.read());
        Serial3.print("Read it");    
    }
    //};
    Serial3.println("Command received");
    return (Serial3.readString());
}

// Drive by command function
void drive_by_command(){
    String command = ReadCommand();
    
    switch (command[0]) {
        case 'L':
            turnLeft();            
            break;
        case 'R':
            turnRight();
            break;
        case 'F':
            forward();
            break;
    }

}
// Odomotry
float x = x0;
float y = y0;
float h = h0;
// Last wheel positions.
float lastLPos = 0;
float lastRPos = 0;

void update() {
    noInterrupts();
    float leftValue = l_encoder.position;
    float rightValue = r_encoder.position;
    interrupts();
    float tL = leftValue-lastLPos;  // Change in left wheel position.
    float tR = rightValue-lastRPos;  // Change in right wheel position.
    //const float delta_h = (R/(2*L))*(tR-tL);
    float delta_s = (R/2)*(tR+tL);
//    if (imu.dataReady()) {
//      //delay(100);
//      imu.read();
//      
//     Serial3.println(imu.yaw());           
//    }
    
    //Serial3.println(h);
     // noInterrupts();
     // h = protected_h -h0;
      x = x + delta_s* cos(h);
      y = y + delta_s* sin(h);
        lastLPos = leftValue;
    lastRPos = rightValue;
  //interrupts();
    
    //h = h + delta_h;

    //delay(1000);
    

}

// Convert to cell and show
void CellPosShow(){  
  int row = 0;
  int col = 0;
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
  Serial3.print("[");
  Serial3.print(row);
  Serial3.print(",");
  Serial3.print(col);
  Serial3.print(",");
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


void calculateHeading (float heading_value) {
  
  if (heading_value < PI/4 && heading_value >= -PI/4) {
    Serial3.print("N");
  } else if (heading_value < 3*PI/4 && heading_value >= PI/4) {
    Serial3.print("E");
  } else if (heading_value < PI && heading_value > 3*PI/4 || heading_value < -3*PI/4 && heading_value > -PI) {
    Serial3.print("S");
  } else if (heading_value < -PI/4 && heading_value >= -3*PI/4) {
    Serial3.print("W");
  }
  
}



void detectWalls() {
  float front;
  float left;
  float right;

  maf_sensor.sample();
  float filtered_dis = maf_sensor.value();
  if (maf_sensor.isReady()) {
    front = filtered_dis;
  }
  left = sensor0.readRangeContinuousMillimeters();
  right = sensor1.readRangeContinuousMillimeters();
  
  if (front <150) {Serial3.println("F:1");  }
  else{Serial3.println("F:1");}
  if (left < 150) {Serial3.println("Left wall detected");  }  
  if (right < 150) {Serial3.println("Right wall detected");  }
}



void setup() {   
    //Set up serial and wire and bluetooth
    Serial.begin(9600);
    Serial3.begin(9600);
    Wire.begin();
    //Set up motor
    r_motor.setPWM(0);
    l_motor.setPWM(0);
    delay(500);
    //Set up Encoder
    //r_encoder.position = 0;
    //
    //l_encoder.position = 0; 
    //Set up Robot
//     robot.x = robot.x0;
//     robot.y = robot.y0;
//     robot.h = robot.h0;
    // Set up Lidar
    pinMode(enablePin0,OUTPUT);
    pinMode(enablePin1,OUTPUT);
    digitalWrite(enablePin0, LOW);
    digitalWrite(enablePin1, LOW); 
    delay(1000); 
    // Sensor0
    Serial3.println("Start Sensor 0");
    digitalWrite(enablePin0, HIGH);
    delay(50);
    sensor0.init();
    sensor0.configureDefault();
    sensor0.setAddress(address0);
    Serial3.println(sensor0.readReg(0x212),HEX); // read I2C address
    sensor0.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
    sensor0.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
    sensor0.setTimeout(500);
    sensor0.stopContinuous();
    sensor0.setScaling(RANGE); // configure range or precision 1, 2 oder 3 mm
    delay(300);
    sensor0.startInterleavedContinuous(100);
    delay(100);

    // Sensor1
    Serial3.println("Start Sensor 1");
    digitalWrite(enablePin1, HIGH);
    delay(50);
    sensor1.init();
    sensor1.configureDefault();
    sensor1.setAddress(address1);
    Serial3.println(sensor1.readReg(0x212),HEX);
    sensor1.writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 30);
    sensor1.writeReg16Bit(VL6180X::SYSALS__INTEGRATION_PERIOD, 50);
    sensor1.setTimeout(500);
    sensor1.stopContinuous();
    sensor1.setScaling(RANGE);
    delay(300);
    sensor1.startInterleavedContinuous(100);
    delay(100);
    Serial3.println("Sensors ready! Start reading sensors in 3 seconds ...!");
    //delay(3000);
    // Set up Sensor
    
    //Set up IMU
    //Wire.begin();  // This must be called before IMU::begin().

    // imu.begin();
    // imu.calibrateAcceleration();
    // while (imu.dataReady())
    //     ;
    // imu.reset();


    //Set up LCD
//    lcd.begin(16, 2);              // Number of chars in a row and how many rows.
//    lcd.configureBacklightPin(3);  // Backlight brightness (0 is off).
//    lcd.backlight();               // Set backlight.
//    lcd.clear();                   // Clear display of text.
//    lcd.setCursor(0,0);
//    lcd.print(0);
//    lcd.print(0);

    // Show it's all set
    Serial3.println("All good, ready to go");
    delay(500);
}

void loop() {
  // Testing lidars
     detectWalls();
    while(1){};
    // Get command from bluetooth
    // Move with command
      //   delay(200);
     //update();
      // noInterrupts();
      
     if (imu.dataReady()) {
      imu.read();
      while(isnan(imu.yaw())){imu.read();
      Serial3.println("No reading");};
      noInterrupts();
      const float protected_h = imu.yaw();
      interrupts();
      h = protected_h -h0;
       };
      noInterrupts();
      update();
    //const float leftValue = 1;
    //const float rightValue =2;

    //const float tL = leftValue-lastLPos;  // Change in left wheel position.
    //const float tR = rightValue-lastRPos;  // Change in right wheel position.
    //const float delta_h = (R/(2*L))*(tR-tL);
    //const float delta_s = (R/2)*(tR+tL);

      
      //x = x + delta_s* cos(h);
      //y = y + delta_s* sin(h);
      //lastLPos = leftValue;
      //lastRPos = rightValue;
      interrupts();
  
  Serial3.println(h);
  
     

//    if (imu.dataReady()) {
//      //delay(100);
//      imu.read();
//      
//     Serial3.println(imu.yaw());           
//    }
    
    //Serial3.println(h);

    
//    drive_by_command(); 
//    CellPosShow();
//    delay(500);
    //calculateHeading(h);
    //LCD_Pose();
    // Detect wall
    //detectWalls();
    //LCD_Detect();

    

    // Get positions
    // X = Robot.x;
    // Y = Robot.y;
    // H = Robot.h;
    // Print out positions with LCD


    // Print out positions with LCD

}
