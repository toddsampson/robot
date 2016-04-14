#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float last_ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

ros::NodeHandle  nh;
sensor_msgs::Imu imu;
ros::Publisher imu_pub("imu/data", &imu);

unsigned long last_update = 0;


void setup() {
    nh.initNode();
    nh.advertise(imu_pub);
    
    // initialize device
    nh.loginfo("Initializing I2C devices...\n");
    mpu.initialize();

    // verify connection
    nh.loginfo("Testing device connections...\n");
    nh.loginfo(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

    // load and configure the DMP
    nh.loginfo("Initializing DMP...\n");
    devStatus = mpu.dmpInitialize();
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        nh.loginfo("Enabling DMP...\n");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        nh.loginfo("DMP ready!\n");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        nh.logerror("DMP Initialization failed\n");
    }
}

void loop() {
    if (!dmpReady) return;

    fifoCount = mpu.getFIFOCount();

    if (fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        nh.logwarn("FIFO overflow!\n");

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (fifoCount >= 42) {
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        imu.header.frame_id = "imu";
        imu.header.stamp = nh.now();
        unsigned long now = millis();
        unsigned long dt_r = now - last_update;
        last_update = now;

        // display quaternion values in easy matrix form: w x y z
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        imu.orientation.x = q.x;
        imu.orientation.y = q.y;
        imu.orientation.z = q.z;
        imu.orientation.w = q.w;

        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        float yaw_ang_vel = ypr[0] / (dt_r * 1000);
        float pitch_ang_vel = ypr[1] / (dt_r * 1000);
        float roll_ang_vel = ypr[2] / (dt_r * 1000);

        imu.angular_velocity.x = roll_ang_vel;
        imu.angular_velocity.y = pitch_ang_vel;
        imu.angular_velocity.z = yaw_ang_vel;

        // display initial world-frame acceleration, adjusted to remove gravity
        // and rotated based on known orientation from quaternion
        mpu.dmpGetAccel(&aaReal, fifoBuffer);
        mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
        imu.linear_acceleration.x = aaWorld.x;
        imu.linear_acceleration.y = aaWorld.y;
        imu.linear_acceleration.z = aaWorld.z;

        imu_pub.publish(&imu);
    }
    nh.spinOnce();
    delay(100);
}

