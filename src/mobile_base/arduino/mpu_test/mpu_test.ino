#include <Arduino.h>

#include <I2Cdev.h>
#include <MPU6050.h>
MPU6050 accelgyro;
int16_t ax, ay, az, gx, gy, gz;

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
ros::NodeHandle  nh;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu/data_raw", &imu_msg);

void setup() {
 nh.initNode();
  nh.advertise(imu_pub);

  accelgyro.initialize();
}

void loop() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  imu_msg.header.stamp = nh.now();
  imu_msg.header.frame_id = "imu";

//  imu_msg.orientation.x = 0.0;
//  imu_msg.orientation.y = 0.0;
//  imu_msg.orientation.z = 0.0;
//  imu_msg.orientation.w = 0.0;
////  imu_msg.orientation_covariance = {-1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
//  imu_msg.angular_velocity.x = gx;
//  imu_msg.angular_velocity.y = gy;
//  imu_msg.angular_velocity.z = gz;
////  imu_msg.angular_velocity_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
//  imu_msg.linear_acceleration.x = ax;
//  imu_msg.linear_acceleration.y = ay;
//  imu_msg.linear_acceleration.z = az;
////  imu_msg.linear_acceleration_covariance = {0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0};

  float ax_f, ay_f, az_f, gx_f, gy_f, gz_f; 
  ax_f =((float) ax) / (16384 / 9.807); // 2g scale in m/s^2
  ay_f =((float) ay) / (16384 / 9.807); // 2g scale in m/s^2
  az_f =((float) az) / (16384 / 9.807); // 2g scale in m/s^2
  gx_f=((float) gx) / 16.4f; // for degrees/s 2000 scale
  gy_f=((float) gy) / 16.4f; // for degrees/s 2000 scale
  gz_f=((float) gz) / 16.4f; // for degrees/s 2000 scale
  
  imu_msg.linear_acceleration.x=-ax_f;
  imu_msg.linear_acceleration.y=ay_f;
  imu_msg.linear_acceleration.z=az_f;
  imu_msg.angular_velocity.x=gx_f;
  imu_msg.angular_velocity.y=gy_f;
  imu_msg.angular_velocity.z=gz_f;

  imu_pub.publish( &imu_msg );
  nh.spinOnce();
}

