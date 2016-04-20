#include <ros.h>
ros::NodeHandle  nh;

void setup() {
  // put your setup code here, to run once:
  nh.initNode();
}

void loop() {
  // put your main code here, to run repeatedly:
  ROS_INFO("Testing");
}
