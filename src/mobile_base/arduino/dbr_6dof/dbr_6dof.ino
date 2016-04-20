#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
//#include <Wire.h>

ros::NodeHandle  nh;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu/data", &imu_msg);

FreeSixIMU sixDOF;

void setup() {
//  Wire.begin();

  nh.initNode();
  nh.advertise(imu_pub);
  delay(5);
  sixDOF.init(); //init the Acc and Gyro
  delay(5);
}


void loop()
{
  imu_msg.header.stamp = nh.now();

  float imu_raw[6];
  sixDOF.getValues(imu_raw);
  imu_msg.linear_acceleration.x = imu_raw[0];
  imu_msg.linear_acceleration.y = imu_raw[1];
  imu_msg.linear_acceleration.z = imu_raw[2];

  imu_msg.angular_velocity.x = imu_raw[3];
  imu_msg.angular_velocity.y = imu_raw[4];
  imu_msg.angular_velocity.z = imu_raw[5];

  float imu_quaternation[4];
  sixDOF.getQ(imu_quaternation);
  imu_msg.orientation.x = imu_quaternation[0];
  imu_msg.orientation.y = imu_quaternation[1];
  imu_msg.orientation.z = imu_quaternation[2];
  imu_msg.orientation.w = imu_quaternation[3];

  imu_pub.publish(&imu_msg);

  nh.spinOnce();
  delay(100);
}

