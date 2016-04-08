#include "signswise_encoder.h"

SignswiseEncoder::SignswiseEncoder(int pinA, int pinB, void(*interruptDispatch)()) {
  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinA), interruptDispatch, RISING);
  _pinA = pinA;
  _pinB = pinB;
  
}

void SignswiseEncoder::encoderISR(void) {
  if (digitalRead(_pinA) == digitalRead(_pinB)) _encoderPulseCount++;
  else _encoderPulseCount--;
}

long SignswiseEncoder::getEncoderCount(void) {
  return _encoderPulseCount;
}

// Thanks to all the code and comments posted online!  Expecially:
// https://github.com/buuav/hackathon1/tree/397b11608ca5c6d7549c1878ce0c4237108b9ffc/Components/motor_driver
// https://community.particle.io/t/cpp-attachinterrupt-to-class-function-help-solved/5147/2
// https://github.com/sungjik/my_personal_robotic_companion
// TODO Add this to readme or a credits/thanks file
