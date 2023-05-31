#include "JoystickHandle.h"

// Joystick I2C address
JoystickHandle myJoystickHandle(JOYSTICK_I2C_ADDR);

struct Motor {
  const int enablePin;
  const int directionPin;
  const int pwmPin;
  int speed;
};

Motor motor1 = {13, 8, 10, 0};
Motor motor2 = {12, 7, 9, 0};
Motor motor3 = {11, 6, 5, 0}; 

const int sFineTune = 128;
const int tFineTune = 200;
unsigned long previousMillis = 0; 
const long interval = 100; 

void setupMotor(Motor &motor) {
  pinMode(motor.enablePin, OUTPUT);
  pinMode(motor.directionPin, OUTPUT);
  pinMode(motor.pwmPin, OUTPUT);
}

void setup() {
  Serial.begin(9600);
  setupMotor(motor1);
  setupMotor(motor2);
  setupMotor(motor3);
}

void updateMotor(Motor &motor, bool direction, int speed) {
  digitalWrite(motor.directionPin, direction);
  analogWrite(motor.enablePin, speed);
}

void loop() {
  unsigned long currentMillis = millis();

  if(currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    int joystickValueY = myJoystickHandle.AnalogRead_Y();
    int joystickValueX = myJoystickHandle.AnalogRead_X();

    Serial.println(joystickValueY);
    Serial.println(joystickValueX);

    // Handle Motor1 and Motor2
    if (joystickValueY > 128) {
      motor1.speed = motor2.speed = map(joystickValueY, 128, 255, 0, 255-sFineTune);
      updateMotor(motor1, HIGH, motor1.speed);
      updateMotor(motor2, HIGH, motor2.speed);
    } else if (joystickValueY < 128) {
      motor1.speed = motor2.speed = map(joystickValueY, 0, 128, 255-sFineTune, 0);
      updateMotor(motor1, LOW, motor1.speed);
      updateMotor(motor2, LOW, motor2.speed);
    } else {
      updateMotor(motor1, LOW, 0);
      updateMotor(motor2, LOW, 0);
    }

    if (joystickValueX < 128) {
      int xMapped = map(joystickValueX, 128, 0, 0, 255-tFineTune);
      motor1.speed = constrain(motor1.speed - xMapped, 0, 255);
      motor2.speed = constrain(motor2.speed + xMapped, 0, 255);
    } else if (joystickValueX > 128) {
      int xMapped = map(joystickValueX, 128, 255, 0, 255-tFineTune);
      motor1.speed = constrain(motor1.speed + xMapped, 0, 255);
      motor2.speed = constrain(motor2.speed - xMapped, 0, 255);
    }

    analogWrite(motor1.pwmPin, motor1.speed);
    analogWrite(motor2.pwmPin, motor2.speed);

    
    if (myJoystickHandle.Get_Button_Status(BUTOON_UP) == PRESS_DOWN) {
      updateMotor(motor3, HIGH, 255); // Assuming 255 is full speed
      Serial.println("BUTOON_UP Pressed, Motor3 is spinning");
    }
    if (myJoystickHandle.Get_Button_Status(BUTOON_RIGHT) == PRESS_DOWN) {
      updateMotor(motor3, LOW, 0);
      Serial.println("BUTOON_RIGHT Pressed, Motor3 stopped");
    }
  }
}
