#include "mpu6050_imu.h"


void imuDispatch();
Mpu6050Imu imu(0x68, 2, &imuDispatch);
void imuDispatch() { imu.dmpDataReady(); }


void setup() {
}

void loop() {

  delay(1);
}
