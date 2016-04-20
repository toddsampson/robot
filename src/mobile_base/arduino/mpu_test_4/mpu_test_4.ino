// Arduino code handles all ros publishing based on data from the various service wrappers
// May want to update this to fully independent publishers in the future using a multi-threaded timer
// Maybe make an array of published/subscribed topics for ros in the main settings include and loop through those on setup/publish
// Allow publish frequency to be passed into each service as a variable to setting loop/callback?

// Setup ROS
#include <ros.h>
#include <ros/time.h>
ros::NodeHandle  nh;

// Setup MPU6050 IMU
#include "mpu6050_imu.h"
#include <sensor_msgs/Imu.h>
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu/data", &imu_msg);
void imuDispatch();
Mpu6050Imu imu(0x68, 2, &imuDispatch);
void imuDispatch() { imu.dmpDataReady(); }


void setup() {
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
