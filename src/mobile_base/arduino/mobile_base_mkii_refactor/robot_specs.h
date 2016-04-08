#ifndef ROBOT_SPECS_H
  #define ROBOT_SPECS_H

  #include "mega_moto.h"
  #include "signswise_encoder.h"

  float wheelDiameter = 20.32; // In cm (8 in)
  byte wheelSeparation = 48.26; // In cm (19 in)
  int encoderTicks = 1680; // Per rotation
  byte gearRatio =  1; //(10:28)

  // Motor Driver: Mega Moto
  MegaMoto leftMotor(10, 9, 8);
  MegaMoto rightMotor(6, 5, 8);

  // Wheel Encoders: Signswise
  void leftEncoderDispatch(); // void leftMotorISRDispatch(), rightMotorISRDispatch(); 
  SignswiseEncoder leftEncoder(20, 21, &leftEncoderDispatch);
  void leftEncoderDispatch() { leftEncoder.encoderISR(); }

#endif
