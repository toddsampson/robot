#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <FreeSixIMU.h>

ros::NodeHandle  nh;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu/data", &imu_msg);

void setup() {
  nh.initNode();
  nh.advertise(imu_pub);
}


void loop()
{
  imu_msg.header.stamp = nh.now();

  imu_msg.linear_acceleration.x = 0;
  imu_msg.linear_acceleration.y = 0;
  imu_msg.linear_acceleration.z = 0;

  imu_msg.angular_velocity.x = 0;
  imu_msg.angular_velocity.y = 0;
  imu_msg.angular_velocity.z = 0;

  imu_msg.orientation.x = 0;
  imu_msg.orientation.y = 0;
  imu_msg.orientation.z = 0;
  imu_msg.orientation.w = 0;

  imu_pub.publish(&imu_msg);

  nh.spinOnce();
  delay(100);
}

