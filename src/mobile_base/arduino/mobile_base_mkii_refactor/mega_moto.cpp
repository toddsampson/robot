#include "mega_moto.h"

MegaMoto::MegaMoto(int forwardPin, int reversePin, int enablePin) {
  pinMode(enablePin,OUTPUT);
  digitalWrite(enablePin,HIGH); // Enable motors
  _forwardPin = forwardPin;
  _reversePin = reversePin;
  stop(); // Stop motors
}

void MegaMoto::forward(int pwm) {
  analogWrite(_forwardPin, pwm);
  analogWrite(_reversePin, 0);
}

void MegaMoto::backward(int pwm) {
  analogWrite(_forwardPin, 0);
  analogWrite(_reversePin, pwm);
}

void MegaMoto::stop(void) {
  analogWrite(_forwardPin, 0);
  analogWrite(_reversePin, 0);
}
