#ifndef MEGA_MOTO_H
  #define MEGA_MOTO_H

  #include <Arduino.h>

  class MegaMoto {
    public:
      MegaMoto(int forwardPin, int reversePin, int enablePin);
      void forward(int pwm);
      void backward(int pwm);
      void stop(void);

    private:
      int _forwardPin;
      int _reversePin;
  };

#endif

