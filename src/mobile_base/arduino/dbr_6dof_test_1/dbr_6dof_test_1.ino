// # Editor     : Roy from DFRobot
// # Date       : 10.12.2013
// # Product name: 6 Dof shield for Arduino
// # Product SKU : DFR0209
// # Version     : 0.1
// # Description:
// # The sketch for driving the 6 Dof shield for Arduino via I2C interface
 
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>
 
#include <Wire.h>
 
int16_t angle[2]; // pitch & roll
 
// Set the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();
int rawSixDof[6];
void setup() 
{ 
  Serial.begin(9600);
  Wire.begin();
   
  delay(5);
  sixDOF.init();                        //begin the IMU
  delay(5);
}
 
void loop() { 
     
  sixDOF.getRawValues(rawSixDof);
  for (int i=0; i<6; i++)              //output the raw data
  {
    switch (i)
    {
        case 0:
      Serial.print("Acc.x :");
      break;
      case 1:
        Serial.print("Acc.y :");
        break;
      case 2:
        Serial.print("Acc.z :");
        break;
      case 3:
        Serial.print("gyro.x :");
        break;
      case 4:
        Serial.print("gyro.y :");
        break;
      case 5:
        Serial.print("gyro.z :");
      break;
        default:
            Serial.print("Err");
    }
    Serial.println(rawSixDof[i]);
  }
  Serial.println("");
  angle[0] = _atan2(rawSixDof[0],rawSixDof[2]);
  angle[1] = _atan2(rawSixDof[1],rawSixDof[2]);
   
  Serial.print("X:");              //pitch & roll
  Serial.println(angle[0]/10.0);
  Serial.print("Y:");
  Serial.println(angle[1]/10.0);
  Serial.println("");
  delay(1000); 
}
 
int16_t _atan2(int32_t y, int32_t x)   //get the _atan2
{ 
  float z = (float)y / x;
  int16_t a;
  if ( abs(y) < abs(x) )
  {
    a = 573 * z / (1.0f + 0.28f * z * z);
    if (x<0)
    {
    if (y<0) a -= 1800;
    else a += 1800;
    }
  }
  else 
  {
    a = 900 - 573 * z / (z * z + 0.28f);
    if (y<0) a -= 1800;
  }
  return a;
}
