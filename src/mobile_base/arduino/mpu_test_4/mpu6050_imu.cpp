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
    // TODO: Return error for ros logging to pass back
  }
  _mpu = mpu;
}

void Mpu6050Imu::dmpDataReady(void) {
//    _mpuInterrupt = true;
  if (!_dmpReady) return;

  while (!_mpuInterrupt && _fifoCount < _packetSize) {
    // wait for MPU interrupt or extra patckets
  }

  _mpuInterrupt = false; // reset interrupt flag
  _mpuIntStatus = _mpu.getIntStatus(); // get INT_STATUS byte
  _fifoCount = _mpu.getFIFOCount(); // get current FIFO count

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((_mpuIntStatus & 0x10) || _fifoCount == 1024) {
    _mpu.resetFIFO();

  //otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (_mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (_fifoCount < _packetSize) _fifoCount = _mpu.getFIFOCount();

    // read a packet from FIFO
    _mpu.getFIFOBytes(_fifoBuffer, _packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    _fifoCount -= _packetSize;

    _temp = _mpu.getTemperature() / 340.00 + 36.53; // get temperature
    _mpu.dmpGetQuaternion(&_q, _fifoBuffer); // get quaternion
    _mpu.dmpGetEuler(_euler, &_q);
    _mpu.dmpGetGravity(&_gravity, &_q);
    _mpu.dmpGetYawPitchRoll(_ypr, &_q, &_gravity);
    _mpu.dmpGetAccel(&_aa, _fifoBuffer);
    _mpu.dmpGetLinearAccel(&_aaReal, &_aa, &_gravity);
    _mpu.dmpGetLinearAccelInWorld(&_aaWorld, &_aaReal, &_q);


   
  }

}





