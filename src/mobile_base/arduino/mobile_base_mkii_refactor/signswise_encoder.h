#ifndef SIGNSWISE_ENCODER_H
  #define SIGNSWISE_ENCODER_H

  #include <Arduino.h>

  class SignswiseEncoder {
    public:
      SignswiseEncoder(int pinA, int pinB, void(*interruptDispatch)());
      void encoderISR(void);
      long getEncoderCount(void);

    private:
      int _pinA;
      int _pinB;
      volatile long _encoderPulseCount;
  };

#endif
