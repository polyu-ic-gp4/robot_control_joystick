#include "JoystickHandle.h"

JoystickHandle myJoystickHandle(JOYSTICK_I2C_ADDR);

const int enB = 13, in3 = 8, in4 = 10, enA = 12, in1 = 7, in2 = 9, inr = 26;
int motorSpeed = 0, motorSpeed2 = 0;
const int sFineTune = 128, tFineTune = 200;

bool isRelayOn = false;  
bool isButtonPressedBefore = false;

void setupMotorControlPins() {
  const int controlPins[] = {enB, in3, in4, enA, in1, in2, inr};
  for (int pin : controlPins) {
    pinMode(pin, OUTPUT);
  }
}

void setup() {
  Serial.begin(9600);
  setupMotorControlPins();
}

void controlMotors(int joystickValueY, int joystickValueX) {
  bool forwardDirection = joystickValueY > 128;
  digitalWrite(in3, forwardDirection ? HIGH : LOW);
  digitalWrite(in4, forwardDirection ? LOW : HIGH);
  digitalWrite(in2, forwardDirection ? HIGH : LOW);
  digitalWrite(in1, forwardDirection ? LOW : HIGH);

  int start = forwardDirection ? 128 : 0;
  int end = forwardDirection ? 255 : 128;
  motorSpeed = map(joystickValueY, start, end, 0, 255 - sFineTune);
  motorSpeed2 = map(joystickValueY, start, end, 0, 255 - sFineTune);

  int xMapped;
  if (joystickValueX < 128) {
    xMapped = map(joystickValueX, 128, 0, 0, 255 - tFineTune);
    motorSpeed -= xMapped;
    motorSpeed2 += xMapped;
  } else if (joystickValueX > 128) {
    xMapped = map(joystickValueX, 128, 255, 0, 255 - tFineTune);
    motorSpeed += xMapped;
    motorSpeed2 -= xMapped;
  }

  motorSpeed = constrain(motorSpeed, 0, 255);
  motorSpeed2 = constrain(motorSpeed2, 0, 255);
}

void loop() {
  int joystickValueY = myJoystickHandle.AnalogRead_Y();
  int joystickValueX = myJoystickHandle.AnalogRead_X();
  
  Serial.println(joystickValueY);
  Serial.println(joystickValueX);

  controlMotors(joystickValueY, joystickValueX);

  bool isButtonPressedNow = myJoystickHandle.Get_Button_Status(BUTOON_RIGHT) == PRESS_DOWN;
  if (isButtonPressedNow && !isButtonPressedBefore) {
    isRelayOn = !isRelayOn;  
  }
  digitalWrite(inr, isRelayOn ? HIGH : LOW);
  isButtonPressedBefore = isButtonPressedNow;  

  analogWrite(enA, motorSpeed2);
  analogWrite(enB, motorSpeed);

  delay(100);
}
