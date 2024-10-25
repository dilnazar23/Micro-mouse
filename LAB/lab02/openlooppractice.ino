// C++ code
//
//#include <Arduino.cpp>
//Motor A: 10backward
int A_PWM = 3;
int A_IN1 = 2;
int A_IN2 = 4;

//Motor B: 10forward
int B_PWM = 6;
int B_IN1 = 5;
int B_IN2 = 7;
void setup()
{
  pinMode(A_PWM, OUTPUT);
  pinMode(A_IN1, OUTPUT);
  pinMode(A_IN2, OUTPUT);
  pinMode(B_PWM, OUTPUT);
  pinMode(B_IN1, OUTPUT);
  pinMode(B_IN2, OUTPUT);
}

void loop()
{
  digitalWrite(B_IN1, HIGH);
  digitalWrite(B_IN2, LOW);
  analogWrite(B_PWM, 100);
  while(B_PWM!=0){
    pause 100;
    digitalWrite(B_IN1, LOW);
    digitalWrite(B_IN2, LOW);    
  }
  //delay(1000); // Wait for 1000 millisecond(s)
  //digitalWrite(LED_BUILTIN, LOW);
  //delay(1000); // Wait for 1000 millisecond(s)
}