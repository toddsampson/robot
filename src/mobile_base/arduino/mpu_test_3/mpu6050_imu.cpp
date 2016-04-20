#include "mpu6050_imu.h"

Mpu6050Imu::Mpu6050Imu(uint8_t imuAddress, int intPin, void(*interruptDispatch)()) {
  MPU6050 mpu(imuAddress);
  pinMode(intPin, INPUT); // Arduino Interrupt

  Wire.begin();

  mpu.initialize();
  _devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  if (_devStatus == 0) {
    mpu.setDMPEnabled(true); // turn on dmp
    attachInterrupt(digitalPinToInterrupt(intPin),  interruptDispatch, RISING); // enable Arduino interrupt detection
    _mpuIntStatus = mpu.getIntStatus();
    _dmpReady = true;
    _packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    nh.logerror("MPU6050 IMU not initialized");
  }

  nh.initNode();
  nh.advertise(imu_pub);
}

void Mpu6050Imu::dmpDataReady(void) {
    _mpuInterrupt = true;
}

void Mpu6050Imu::publishImu(void) {
  if (!_dmpReady) return;

  while (!_mpuInterrupt && _fifoCount < _packetSize) {
    // wait for MPU interrupt or extra patckets
  }

  _mpuInterrupt = false; // reset interrupt flag
  _mpuIntStatus = mpu.getIntStatus(); // get INT_STATUS byte
  _fifoCount = mpu.getFIFOCount(); // get current FIFO count

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((_mpuIntStatus & 0x10) || _fifoCount == 1024) {
    mpu.resetFIFO();

  }
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  //  } else if (mpuIntStatus & 0x02) {
//    // wait for correct available data length, should be a VERY short wait
//    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
//
//    // read a packet from FIFO
//    mpu.getFIFOBytes(fifoBuffer, packetSize);
//
//    // track FIFO count here in case there is > 1 packet available
//    // (this lets us immediately read more without waiting for an interrupt)
//    fifoCount -= packetSize;
//
//    temp = mpu.getTemperature() / 340.00 + 36.53; // get temperature
//    mpu.dmpGetQuaternion(&q, fifoBuffer); // get quaternion
//    mpu.dmpGetEuler(euler, &q);
//    mpu.dmpGetGravity(&gravity, &q);
//    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//    mpu.dmpGetAccel(&aa, fifoBuffer);
//    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
//    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
//
//    imu_msg.header.stamp = nh.now();
//    imu_msg.header.frame_id = "imu";
//
//    imu_msg.orientation.x = q.x;
//    imu_msg.orientation.y = q.y;
//    imu_msg.orientation.z = q.z;
//    imu_msg.orientation.w = q.w;
//    // imu_msg.orientation_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
//    // TODO: Fix this for time???
//    imu_msg.angular_velocity.x = aaWorld.x;
//    imu_msg.angular_velocity.y = aaWorld.y;
//    imu_msg.angular_velocity.z = aaWorld.z;
//    // imu_msg.angular_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
//    imu_msg.linear_acceleration.x = aaWorld.x;
//    imu_msg.linear_acceleration.y = aaWorld.y;
//    imu_msg.linear_acceleration.z = aaWorld.z;
//    // imu_msg.linear_acceleration_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
//
//    imu_pub.publish( &imu_msg );
//  }
//  nh.spinOnce();
  
}




