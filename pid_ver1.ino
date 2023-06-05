#include "JoystickHandle.h"

JoystickHandle myJoystickHandle(JOYSTICK_I2C_ADDR);

int enB = 13;   
int in3 = 8;    
int in4 = 10;   
int motorSpeed = 0;
int targetSpeed1 = 0;
double kp1 = 0.5;  
double ki1 = 0.1;  
double kd1 = 0.2; 
double integral1 = 0;
double previousError1 = 0;
int enA = 12;   
int in1 = 7;    
int in2 = 9;    
int motorSpeed2 = 0;
int targetSpeed2 = 0;
double kp2 = 0.5;  
double ki2 = 0.1;
double kd2 = 0.2;  
double integral2 = 0;
double previousError2 = 0;

int sLimiter = 128;
int tLimiter = 200;
int rLimiter = 30;
int lLimiter = 0;
int rm = 48;
int in5 = 49;
int in6 = 11;
int motorSpeed3 = 0;
bool buttonRStatus = false;
bool buttonLStatus = false;
bool buttonRLatch = false;
bool buttonLLatch = false;

void setup() {
  Serial.begin(9600);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(rm, OUTPUT);
  pinMode(in5, OUTPUT);
  pinMode(in6, OUTPUT);
}

void motorControl(int joystickValue, int &motorSpeed, int targetSpeed, double kp, double ki, double kd, double &integral, double &previousError) {
  int error = targetSpeed - motorSpeed;
  integral += error;
  double derivative = error - previousError;
  previousError = error;

  // Apply PID control
  int controlSignal = kp * error + ki * integral + kd * derivative;

  // Update motor speed
  motorSpeed += controlSignal;
  motorSpeed = constrain(motorSpeed, 0, 255);
}

void loop() {
  int joystickValueY = myJoystickHandle.AnalogRead_Y(); 
  int joystickValueX = myJoystickHandle.AnalogRead_X();

 
  if (joystickValueY > 128) {  // Forward direction
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in1, LOW);
    targetSpeed1 = map(joystickValueY, 128, 255, 0, 255 - sLimiter - lLimiter);
    targetSpeed2 = map(joystickValueY, 128, 255, 0, 255 - sLimiter - rLimiter);
  } else if (joystickValueY < 128) {  
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in1, HIGH);
    targetSpeed1 = map(joystickValueY, 0, 128, 255 - sLimiter - lLimiter, 0);
    targetSpeed2 = map(joystickValueY, 0, 128, 255 - sLimiter - rLimiter, 0);
  } else {  // Stop
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  // Apply PID control to motors 1 and 2
  motorControl(joystickValueY, motorSpeed, targetSpeed1, kp1, ki1, kd1, integral1, previousError1);
  motorControl(joystickValueY, motorSpeed2, targetSpeed2, kp2, ki2, kd2, integral2, previousError2);

  if (joystickValueX < 128) {
    int xMapped = map(joystickValueX, 128, 0, 0, 255 - tLimiter);

    motorSpeed -= xMapped;
    motorSpeed2 += xMapped;
    // Confine the range from 0 to 255
    motorSpeed = constrain(motorSpeed, 0, 255);
    motorSpeed2 = constrain(motorSpeed2, 0, 255);
  } else if (joystickValueX > 128) {
    int xMapped = map(joystickValueX, 128, 255, 0, 255 - tLimiter);

    motorSpeed += xMapped;
    motorSpeed2 -= xMapped;
    // Confine the range from 0 to 255
    motorSpeed = constrain(motorSpeed, 0, 255);
    motorSpeed2 = constrain(motorSpeed2, 0, 255);
  }

  analogWrite(enA, motorSpeed2); // Send PWM signal to motor A
  analogWrite(enB, motorSpeed); // Send PWM signal to motor B

  if (myJoystickHandle.Get_Button_Status(BUTOON_RIGHT) == PRESS_DOWN && !buttonRStatus) {
    buttonRStatus = true;
    buttonRLatch = !buttonRLatch;
    buttonLLatch = false;
  } else if (myJoystickHandle.Get_Button_Status(BUTOON_LEFT) == PRESS_DOWN && !buttonLStatus) {
    buttonLStatus = true;
    buttonLLatch = !buttonLLatch;
    buttonRLatch = false;
  } else if (myJoystickHandle.Get_Button_Status(BUTOON_RIGHT) == NONE_PRESS && buttonRStatus) {
    buttonRStatus = false;
  } else if (myJoystickHandle.Get_Button_Status(BUTOON_LEFT) == NONE_PRESS && buttonLStatus) {
    buttonLStatus = false;
  }
  if (buttonRLatch) {
    motorSpeed3 = 255;
    digitalWrite(in5, HIGH);
    digitalWrite(in6, LOW);
  } else if (buttonLLatch) {
    motorSpeed3 = 255;
    digitalWrite(in5, LOW);
    digitalWrite(in6, HIGH);
  } else {
    motorSpeed3 = 0;
    digitalWrite(in5, LOW);
    digitalWrite(in6, LOW);
  }
  analogWrite(rm, motorSpeed3); // Send PWM signal to motor 3
  delay(100);  // Delay to allow for joystick to be moved
}
