#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
#include <Wire.h>

ros::NodeHandle  nh;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu/data_raw", &imu_msg);

FreeSixIMU sixDOF;

void setup() {
  Wire.begin();
  
  nh.initNode();
  nh.advertise(imu_pub);

  sixDOF.init();
}


void loop() {
  imu_msg.header.stamp = nh.now();

  float imu_quaternation[4];
  sixDOF.getQ(imu_quaternation);
  imu_msg.orientation.x = imu_quaternation[0];
  imu_msg.orientation.y = imu_quaternation[1];
  imu_msg.orientation.z = imu_quaternation[2];
  imu_msg.orientation.w = imu_quaternation[3];

  float imu_euler[3];
  sixDOF.getEuler(imu_euler);
  imu_msg.angular_velocity.x = imu_euler[0];
  imu_msg.angular_velocity.y = imu_euler[1];
  imu_msg.angular_velocity.z = imu_euler[2];

  float imu_accel[3];
  sixDOF.getValues(imu_accel);
  imu_msg.linear_acceleration.x = imu_accel[0];
  imu_msg.linear_acceleration.y = imu_accel[1];
  imu_msg.linear_acceleration.z = imu_accel[2];


  imu_pub.publish(&imu_msg);

  nh.spinOnce();
  delay(100);
}
