#ifndef MPU6050_IMU_H
  #define MPU6050_IMU_H

  #include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
ros::NodeHandle  nh;
#include <sensor_msgs/Imu.h>
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu/data", &imu_msg);

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define INTERRUPT_PIN 2 // Arduino Interrupt

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion/temperature vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float temp;             //                      temperature in celsius

// Interrupt detection
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high


//class Mpu6050Imu {
//  public:
//    Mpu6050Imu(void);
//    void dmpDataReady(void);
//};

//  void imuDispatch();
//Mpu6050Imu imu_int(2, &imuDispatch);
//void imuDispatch() { imu_int.dmpDataReady(); }

#endif
