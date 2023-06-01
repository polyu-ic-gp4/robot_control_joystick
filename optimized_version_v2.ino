#include "JoystickHandle.h"

JoystickHandle myJoystickHandle(JOYSTICK_I2C_ADDR);

struct Motor {
  int en;
  int in1;
  int in2;
  int speed;
};

Motor motors[3] = {
  {13, 8, 10, 0}, // Motor 1
  {12, 7, 9, 0},  // Motor 2
  {48, 49, 11, 0} // Motor 3
};

int sFineTune = 128;
int tFineTune = 200;

bool buttonRStatus = false;
bool buttonLStatus = false;
bool buttonRLatch = false;
bool buttonLLatch = false;

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < 3; i++) {
    pinMode(motors[i].en, OUTPUT);
    pinMode(motors[i].in1, OUTPUT);
    pinMode(motors[i].in2, OUTPUT);
  }
}

void motorControl(int joystickValue, Motor &motor, int motorDirection1, int motorDirection2) {
  digitalWrite(motor.in1, motorDirection1);
  digitalWrite(motor.in2, motorDirection2);
  if (joystickValue > 128) {
    motor.speed = map(joystickValue, 128, 255, 0, 255-sFineTune);
  } else if (joystickValue < 128) {
    motor.speed = map(joystickValue, 0, 128, 255-sFineTune, 0);
  }
  analogWrite(motor.en, motor.speed);
}

void loop() {
  int joystickValueY = myJoystickHandle.AnalogRead_Y();
  int joystickValueX = myJoystickHandle.AnalogRead_X();

  motorControl(joystickValueY, motors[0], HIGH, LOW);
  motorControl(joystickValueY, motors[1], LOW, HIGH);
  if (joystickValueY == 128) {
    digitalWrite(motors[0].in1, LOW);
    digitalWrite(motors[0].in2, LOW);
    digitalWrite(motors[1].in1, LOW);
    digitalWrite(motors[1].in2, LOW);
  }

  int xMapped;
  if (joystickValueX < 128) {
    xMapped = map(joystickValueX, 128, 0, 0, 255-tFineTune);
    motors[0].speed -= xMapped;
    motors[1].speed += xMapped;
  } else if (joystickValueX > 128) {
    xMapped = map(joystickValueX, 128, 255, 0, 255-tFineTune);
    motors[0].speed += xMapped;
    motors[1].speed -= xMapped;
  }
  
  motors[0].speed = constrain(motors[0].speed, 0, 255);
  motors[1].speed = constrain(motors[1].speed, 0, 255);

  analogWrite(motors[0].en, motors[0].speed);
  analogWrite(motors[1].en, motors[1].speed);

  // Check buttons
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

  // Check motor latch
  if (buttonRLatch) {
    digitalWrite(motors[2].in1, HIGH);
    digitalWrite(motors[2].in2, LOW);
    motors[2].speed = 255;
  } else if (buttonLLatch) {
    digitalWrite(motors[2].in1, LOW);
    digitalWrite(motors[2].in2, HIGH);
    motors[2].speed = 255;
  } else {
    digitalWrite(motors[2].in1, LOW);
    digitalWrite(motors[2].in2, LOW);
  }
  analogWrite(motors[2].en, motors[2].speed);
  delay(100);
}
