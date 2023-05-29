#include "JoystickHandle.h"

JoystickHandle myJoystickHandle(JOYSTICK_I2C_ADDR);

// Motor 1 MEGA I/O pin to L298N control pin mapping
int enB = 13;   // L298N control pin for enabling motor 1
int in3 = 8;    // L298N control pin for motor 1 direction control
int in4 = 10;   // L298N control pin for motor 1 PWM input
int motorSpeed = 0;

// Motor 2 MEGA I/O pin to L298N control pin mapping
int enA = 12;   // L298N control pin for enabling motor 1
int in1 = 7;    // L298N control pin for motor 1 direction control
int in2 = 9;   // L298N control pin for motor 1 PWM input
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

void loop() {
  
  int joystickValueY =myJoystickHandle.AnalogRead_Y(); // Reading joystick Y value
  int joystickValueX =myJoystickHandle.AnalogRead_X();
  // Mapping joystickValue (0 to 255) to motorSpeed (0 to 255)
  

  Serial.println(myJoystickHandle.AnalogRead_Y());
  Serial.println(myJoystickHandle.AnalogRead_X());
  // Controlling the direction of the motor based on the joystick value
  if (joystickValueY > 128) {  // Forward direction
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in1, LOW);
    motorSpeed = map(joystickValueY, 128, 255, 0, 255);
    motorSpeed2 = map(joystickValueY, 128, 255, 0, 255);
    analogWrite(enB, motorSpeed);
    analogWrite(enA, motorSpeed2);
  } else if (joystickValueY < 128) {  // Reverse direction is removed as motor can't move in reverse direction
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in1, HIGH);
    motorSpeed = map(joystickValueY, 0, 128, 255, 0);
    motorSpeed2 = map(joystickValueY, 0, 128, 255, 0);
    analogWrite(enB, motorSpeed);
    analogWrite(enA, motorSpeed2);

    } else if (joystickValueY == 128) {  // Reverse direction is removed as motor can't move in reverse direction
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
 
//  } else {  // Stop
//    digitalWrite(in3, LOW);
//    digitalWrite(in4, LOW);
//    motorSpeed = 0; // Set motor speed to 0 for stop
  }
 if (joystickValueX < 128) {

    int xMapped = map(joystickValueX, 128, 0, 0, 255);
    
    motorSpeed = motorSpeed - xMapped;
    motorSpeed2 = motorSpeed2 + xMapped;
    // Confine the range from 0 to 255
    if (motorSpeed < 0) {
      motorSpeed = 0;
    }
    if (motorSpeed2 > 255) {
      motorSpeed2 = 255;
    }
    analogWrite(enA, motorSpeed2); // Send PWM signal to motor A
  analogWrite(enB, motorSpeed); // Send PWM signal to motor B
  }
  if (joystickValueX > 128) {
   
    int xMapped = map(joystickValueX, 128, 255, 0, 255);
    
    motorSpeed = motorSpeed + xMapped;
    motorSpeed2 = motorSpeed2 - xMapped;
  
    if (motorSpeed > 255) {
      motorSpeed = 255;
    }
    if (motorSpeed2 < 0) {
      motorSpeed2 = 0;
    }
    analogWrite(enA, motorSpeed2); // Send PWM signal to motor A
  analogWrite(enB, motorSpeed); // Send PWM signal to motor B
  }
  


  
  delay(100);  // Delay to allow for joystick to be moved
}


