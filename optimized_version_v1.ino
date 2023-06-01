#include "JoystickHandle.h"

JoystickHandle myJoystickHandle(JOYSTICK_I2C_ADDR);

const int enB = 13;   
const int in3 = 8;   
const int in4 = 10;   


const int enA = 12;  
const int in1 = 7;    
const int in2 = 9;    

const int inr = 26;

int motorSpeed = 0, motorSpeed2 = 0;
const int sFineTune = 128, tFineTune = 200;

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

void adjustMotorSpeeds(int xMapped) {
  motorSpeed = constrain(motorSpeed - xMapped, 0, 255);
  motorSpeed2 = constrain(motorSpeed2 + xMapped, 0, 255);
  analogWrite(enA, motorSpeed2); // Send PWM signal to motor A
  analogWrite(enB, motorSpeed); // Send PWM signal to motor B
}

void loop() {
  int joystickValueY = myJoystickHandle.AnalogRead_Y(); // Reading joystick Y value
  int joystickValueX = myJoystickHandle.AnalogRead_X(); // Reading joystick X value

  Serial.println(joystickValueY);
  Serial.println(joystickValueX);

  if (joystickValueY > 128) {  // Forward direction
    digitalWrite(in3,  HIGH);
    digitalWrite(in4, LOW);
    digitalWrite(in2,  HIGH);
    digitalWrite(in1, LOW);
    motorSpeed = motorSpeed2 = map(joystickValueY, 128, 255, 0, 255-sFineTune);
    analogWrite(enB, motorSpeed);
    analogWrite(enA, motorSpeed2);
  } else if (joystickValueY < 128) {  // Reverse direction
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in1, HIGH);
    motorSpeed = motorSpeed2 = map(joystickValueY, 0, 128, 255-sFineTune, 0);
    analogWrite(enB, motorSpeed);
    analogWrite(enA, motorSpeed2);
  } else {  // No motion
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  if (joystickValueX < 128) {
    adjustMotorSpeeds(map(joystickValueX, 128, 0, 0, 255-tFineTune));
  } else if (joystickValueX > 128) {
    adjustMotorSpeeds(map(joystickValueX, 128, 255, 0, 255-tFineTune));
  }
  
  digitalWrite(inr, myJoystickHandle.Get_Button_Status(BUTOON_RIGHT) == PRESS_DOWN ? HIGH : LOW);
}
