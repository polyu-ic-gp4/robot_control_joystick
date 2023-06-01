#include "JoystickHandle.h"

JoystickHandle myJoystickHandle(JOYSTICK_I2C_ADDR);

int enB = 13;   
int in3 = 8;    
int in4 = 10;   
int motorSpeed = 0;


int enA = 12;  
int in1 = 7;    
int in2 = 9;   
int motorSpeed2 = 0;

int sFineTune = 128;
int tFineTune = 200;

int inr = 26;

void setup() {
  Serial.begin(9600);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(inr, OUTPUT);
}

void setDirection(int pin1, int pin2, int state1, int state2) {
  digitalWrite(pin1, state1);
  digitalWrite(pin2, state2);
}

void controlMotorSpeed(int joystickValue, int minVal, int maxVal, int& motorSpeed, int motorPin, int fineTune) {
  motorSpeed = map(joystickValue, minVal, maxVal, 0, 255-fineTune);
  motorSpeed = constrain(motorSpeed, 0, 255);
  analogWrite(motorPin, motorSpeed);
}

void controlTurning(int joystickValue, int minVal, int maxVal, int& motorSpeed, int& motorSpeed2, int pinA, int pinB, int fineTune) {
  int xMapped = map(joystickValue, minVal, maxVal, 0, 255-fineTune);
  
  motorSpeed = motorSpeed + xMapped;
  motorSpeed2 = motorSpeed2 - xMapped;

  motorSpeed = constrain(motorSpeed, 0, 255);
  motorSpeed2 = constrain(motorSpeed2, 0, 255);

  analogWrite(pinA, motorSpeed2);
  analogWrite(pinB, motorSpeed);
}

void loop() {
  int joystickValueY = myJoystickHandle.AnalogRead_Y();
  int joystickValueX = myJoystickHandle.AnalogRead_X();

  if (joystickValueY > 128) {
    setDirection(in3, in4, HIGH, LOW);
    setDirection(in2, in1, HIGH, LOW);
    controlMotorSpeed(joystickValueY, 128, 255, motorSpeed, enB, sFineTune);
    controlMotorSpeed(joystickValueY, 128, 255, motorSpeed2, enA, sFineTune);
  } else if (joystickValueY < 128) {
    setDirection(in3, in4, LOW, HIGH);
    setDirection(in2, in1, LOW, HIGH);
    controlMotorSpeed(joystickValueY, 0, 128, motorSpeed, enB, sFineTune);
    controlMotorSpeed(joystickValueY, 0, 128, motorSpeed2, enA, sFineTune);
  } else {
    setDirection(in3, in4, LOW, LOW);
    setDirection(in1, in2, LOW, LOW);
  }

  if (joystickValueX < 128) {
    controlTurning(joystickValueX, 128, 0, motorSpeed, motorSpeed2, enA, enB, tFineTune);
  }
  
  if (joystickValueX > 128) {
    controlTurning(joystickValueX, 128, 255, motorSpeed, motorSpeed2, enA, enB, tFineTune);
  }

  digitalWrite(inr, myJoystickHandle.Get_Button_Status(BUTOON_RIGHT) == PRESS_DOWN ? HIGH : LOW);
  
  delay(100);
}
