#ifndef MPU6050_IMU_H
  #define MPU6050_IMU_H

  #include <Arduino.h>
  #include "MPU6050_6Axis_MotionApps20.h"
  #include "I2Cdev.h"
  #include "Wire.h"
  #include <ros.h>
  #include <ros/time.h>
  #include <sensor_msgs/Imu.h>

  ros::NodeHandle  nh;
  sensor_msgs::Imu imu_msg;
  ros::Publisher imu_pub("imu/data", &imu_msg);

  class Mpu6050Imu {
    public:
      Mpu6050Imu(uint8_t imuAddress, int intPin, void(*interruptDispatch)());
      void dmpDataReady(void);
      void publishImu(void);
      int getIntStatus(void);
      MPU6050 mpu(uint8_t imuAddress);
    private:
      bool _dmpReady = false;  // set true if DMP init was successful
      bool _mpuInterrupt;
      uint8_t _devStatus; // return status after each device operation (0 = success, !0 = error)
      uint8_t _fifoBuffer[64]; // FIFO storage buffer
      uint8_t _mpuIntStatus; // holds actual interrupt status byte from MPU
      uint16_t _fifoCount; // count of all bytes currently in FIFO
      uint16_t _packetSize; // expected DMP packet size (default is 42 bytes)
  };

  
//  // orientation/motion/temperature vars
//  Quaternion q;           // [w, x, y, z]         quaternion container
//  VectorInt16 aa;         // [x, y, z]            accel sensor measurements
//  VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
//  VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
//  VectorFloat gravity;    // [x, y, z]            gravity vector
//  float euler[3];         // [psi, theta, phi]    Euler angle container
//  float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
//  float temp;             //                      temperature in celsius
//
//  volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

#endif

