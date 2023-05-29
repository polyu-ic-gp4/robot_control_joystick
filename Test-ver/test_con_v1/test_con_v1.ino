#include "JoystickHandle.h"

#define FORWARD HIGH
#define REVERSE LOW
#define PRESS_DOWN 1

JoystickHandle myJoystickHandle(JOYSTICK_I2C_ADDR);


int enB = 13;   
int in3 = 8;    
int in4 = 10;  
int motorSpeed = 0;


int enA = 12;   
int in1 = 7;    
int in2 = 9;    
int motorSpeed2 = 0;

void setup() {
  Serial.begin(9600);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
}

int confineRange(int speed) {
  if (speed < 0) {
    speed = 0;
  } else if (speed > 255) {
    speed = 255;
  }
  return speed;
}

void controlMotor(int joyValue, int &motorSpeed, int inA, int inB, int enA) {
  int direction;
  if (joyValue > 128) {  // Forward direction
    direction = FORWARD;
    motorSpeed = map(joyValue, 128, 255, 0, 255);
  } else if (joyValue < 128) {  
    direction = REVERSE;
    motorSpeed = map(joyValue, 0, 128, 255, 0);
  } else {  // Stop
    direction = REVERSE;
    motorSpeed = 0;
  }

  digitalWrite(inA, direction);
  digitalWrite(inB, !direction);
  motorSpeed = confineRange(motorSpeed);
  analogWrite(enA, motorSpeed);
}

void motorControl(int button, int &motorSpeed, int inA, int inB, int enA, int direction) {
  if (myJoystickHandle.Get_Button_Status(button) == PRESS_DOWN) {
    digitalWrite(inA, direction);
    digitalWrite(inB, !direction);
    motorSpeed = (direction == FORWARD) ? 255 : 0;
    analogWrite(enA, motorSpeed);
  }
}

void loop() {
  int joystickValueY = myJoystickHandle.AnalogRead_Y(); // Reading joystick Y value
  int joystickValueX = myJoystickHandle.AnalogRead_X();

  Serial.println(joystickValueY);
  Serial.println(joystickValueX);

 
  controlMotor(joystickValueY, motorSpeed, in3, in4, enB);
  controlMotor(joystickValueY, motorSpeed2, in1, in2, enA);

  
  motorControl(BUTOON_UP, motorSpeed, in3, in4, enB, FORWARD);
  motorControl(BUTOON_RIGHT, motorSpeed, in3, in4, enB, REVERSE);
  motorControl(BUTOON_DOWN, motorSpeed2, in1, in2, enA, FORWARD);
  motorControl(BUTOON_LEFT, motorSpeed2, in1, in2, enA, REVERSE);

  
  int xMapped;
  if (joystickValueX < 128) {
    xMapped = map(joystickValueX, 128, 0, 0, 255);
    motorSpeed -= xMapped;
    motorSpeed2 += xMapped;
  } else if (joystickValueX > 128) {
    xMapped = map(joystickValueX, 128, 255, 0, 255);
    motorSpeed += xMapped;
    motorSpeed2 -= xMapped;
  }

  motorSpeed = confineRange(motorSpeed);
  motorSpeed2 = confineRange(motorSpeed2);

  analogWrite(enA, motorSpeed2); // Send PWM signal to motor A
  analogWrite(enB, motorSpeed); // Send PWM signal to motor B

  delay(100);  // Delay to allow for joystick to be moved
}
