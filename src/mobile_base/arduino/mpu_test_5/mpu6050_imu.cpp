#include "mpu6050_imu.h"

void Mpu6050Imu::Mpu6050Imu(int intPin, void(*interruptDispatch)()) {
  pinMode(intPin, INPUT);

  Wire.begin();

  mpu.initialize();
  devStatus = mpu.dmpInitialize();

}

void Mpu6050Imu::dmpDataReady(void) {
  if (!_dmpReady) return;

  while (!_mpuInterrupt && _fifoCount < _packetSize) {
    // wait for MPU interrupt or extra patckets
  }

  mpuInterrupt = false; // reset interrupt flag
  mpuIntStatus = mpu.getIntStatus(); // get INT_STATUS byte
  fifoCount = mpu.getFIFOCount(); // get current FIFO count

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    mpu.resetFIFO();

  //otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    temp = mpu.getTemperature() / 340.00 + 36.53; // get temperature
    mpu.dmpGetQuaternion(&q, fifoBuffer); // get quaternion
    mpu.dmpGetEuler(euler, &q);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);


   
  }

}




