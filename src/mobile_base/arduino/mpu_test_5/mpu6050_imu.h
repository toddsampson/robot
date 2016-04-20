#ifndef MPU6050_IMU_H
  #define MPU6050_IMU_H

  #include <Arduino.h>
  #include <ros.h>
  #include <ros/time.h>
  #include <sensor_msgs/Imu.h>
  sensor_msgs::Imu imu_msg;
  ros::Publisher imu_pub("imu/data", &imu_msg);
  ros::NodeHandle  nh;

  #include "MPU6050_6Axis_MotionApps20.h"
  #include "I2Cdev.h"
  #include "Wire.h"


MPU6050 mpu(0x68);      // setup imu on default address



  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip


  class Mpu6050Imu {
    public:
      Mpu6050Imu::Mpu6050_Imu(int intPin, void(*interruptDispatch)());
      void Mpu6050Imu::dmpDataReady(void);
  };

  void imuDispatch();
Mpu6050Imu imu_int(2, &imuDispatch);
void imuDispatch() { imu_int.dmpDataReady(); }

#endif
