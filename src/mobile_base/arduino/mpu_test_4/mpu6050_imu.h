#ifndef MPU6050_IMU_H
  #define MPU6050_IMU_H

  #include <Arduino.h>
  #include "MPU6050_6Axis_MotionApps20.h"
  #include "I2Cdev.h"
  #include "Wire.h"

  class Mpu6050Imu {
    public:
      Mpu6050Imu(uint8_t imuAddress, int intPin, void(*interruptDispatch)());
      void dmpDataReady(void);
      void publishImu(void);
      int getIntStatus(void);
    private:
      MPU6050 _mpu;
      bool _dmpReady = false;              // set true if DMP init was successful
      volatile bool _mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
      uint8_t _devStatus;                  // return status after each device operation (0 = success, !0 = error)
      uint8_t _fifoBuffer[64];             // FIFO storage buffer
      uint8_t _mpuIntStatus;               // holds actual interrupt status byte from MPU
      uint16_t _fifoCount;                 // count of all bytes currently in FIFO
      uint16_t _packetSize;                // expected DMP packet size (default is 42 bytes)
      Quaternion _q;                       // [w, x, y, z]         quaternion container
      VectorInt16 _aa;                     // [x, y, z]            accel sensor measurements
      VectorInt16 _aaReal;                 // [x, y, z]            gravity-free accel sensor measurements
      VectorInt16 _aaWorld;                // [x, y, z]            world-frame accel sensor measurements
      VectorFloat _gravity;                // [x, y, z]            gravity vector
      float _euler[3];                     // [psi, theta, phi]    Euler angle container
      float _ypr[3];                       // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
      float _temp;                         // temperature in celsius
  };

#endif

