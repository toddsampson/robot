
// Setup MPU6050 IMU
#include "mpu6050_imu.h"


void setup() {
  if (devStatus == 0) {
    mpu.setDMPEnabled(true); // turn on dmp
    attachInterrupt(digitalPinToInterrupt(intPin),  interruptDispatch, RISING); // enable Arduino interrupt detection
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // TODO: Return error for ros logging to pass back
  }

  nh.initNode(); // Init ROS
  nh.advertise(imu_pub);
}

void loop() {
  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = "imu";

//  imu_msg.orientation.x = q.x;
//  imu_msg.orientation.y = q.y;
//  imu_msg.orientation.z = q.z;
//  imu_msg.orientation.w = q.w;
//  // imu_msg.orientation_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
//  // TODO: Fix this for time???
//  imu_msg.angular_velocity.x = aaWorld.x;
//  imu_msg.angular_velocity.y = aaWorld.y;
//  imu_msg.angular_velocity.z = aaWorld.z;
//  // imu_msg.angular_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
//  imu_msg.linear_acceleration.x = aaWorld.x;
//  imu_msg.linear_acceleration.y = aaWorld.y;
//  imu_msg.linear_acceleration.z = aaWorld.z;
//  // imu_msg.linear_acceleration_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

  imu_pub.publish( &imu_msg );
  delay(1);
}



