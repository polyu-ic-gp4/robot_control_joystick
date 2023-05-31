#include "JoystickHandle.h"


JoystickHandle myJoystickHandle(JOYSTICK_I2C_ADDR);


struct Motor {
  int enPin;
  int dirPin1;
  int dirPin2;
  int speed;
};



Motor motor1 = {13, 8, 10, 0};
Motor motor2 = {12, 7, 9, 0};
Motor motor3 = {48, 49, 11, 0};

// Button inputs
int buttonUp = A4;
int buttonRight = A5;

void initializeMotor(Motor &motor) {
  pinMode(motor.enPin, OUTPUT);
  pinMode(motor.dirPin1, OUTPUT);
  pinMode(motor.dirPin2, OUTPUT);
}

void updateMotor(Motor &motor, int dir1, int dir2) {
  digitalWrite(motor.dirPin1, dir1);
  digitalWrite(motor.dirPin2, dir2);
}


void setup() {
  // Initialize serial
  Serial.begin(9600);

  // Initialize motor control pins
  initializeMotor(motor1);
  initializeMotor(motor2);
  initializeMotor(motor3);

  // Initialize button pins
  pinMode(buttonUp, INPUT);
  pinMode(buttonRight, INPUT);
}

void loop() {
  // Reading joystick Y value
  int joystickValueY = myJoystickHandle.AnalogRead_Y();

  // Check button states
  if (digitalRead(buttonUp) == LOW) {  // Check if BUTTON_UP is pressed
    digitalWrite(motor3.dirPin1, HIGH);
    digitalWrite(motor3.dirPin2, LOW);
    Serial.println("Button_A Pressed");
  }
  if (digitalRead(buttonRight) == LOW) {  // Check if BUTTON_RIGHT is pressed
    digitalWrite(motor3.dirPin1, LOW);
    digitalWrite(motor3.dirPin2, LOW);
    Serial.println("Button_B Pressed");
  }

  // Controlling the direction of the motor based on the joystick value
  if (joystickValueY > 128) {  // Forward direction
    updateMotor(motor1, HIGH, LOW);
    updateMotor(motor2, HIGH, LOW);
  } else if (joystickValueY < 128) {  // Reverse direction
    updateMotor(motor1, LOW, HIGH);
    updateMotor(motor2, LOW, HIGH);
  } else {  // Stop
    updateMotor(motor1, LOW, LOW);
    updateMotor(motor2, LOW, LOW);
  }
  
  delay(100); 
}
