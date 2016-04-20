const int DesiredRPM=300;
int MotorPWMPin = 3;
int inByte = 0;         // incoming serial byte
unsigned char Data_status=0;
unsigned char Data_4deg_index=0;
unsigned char Data_loop_index=0;
unsigned char SpeedRPHhighbyte=0; // 
unsigned char SpeedRPHLowbyte=0;
int SpeedRPH=0;
const unsigned char PWM4dutyMax=255;
const unsigned char PWM4dutyMin=150;
unsigned char PWM4duty=PWM4dutyMin;

void setup() {
  pinMode(MotorPWMPin, OUTPUT); 
  Serial.begin(115200);  // USB serial
  Serial3.begin(115200);

  analogWrite(MotorPWMPin, 200);
//    analogWrite(MotorPWMPin, PWM4duty);
}

void loop() {
  if (Serial3.available() > 0) {
    inByte = Serial3.read();
    Serial.write(inByte);
    decodeData(inByte);
  }
}

void decodeData(unsigned char inByte){
  switch (Data_status){
  case 0: // no header
    if (inByte==0xFA)
    {
      Data_status=1;
      Data_loop_index=1;
    }
    break;
  case 1: // Find 2nd FA
    if (Data_loop_index==22){
      if (inByte==0xFA)
      {
        Data_status=2;
        Data_loop_index=1;
      } 
      else // if not FA search again
      Data_status=0;
    }
    else{
      Data_loop_index++;
    }
    break;
  case 2: // Read data out
    if (Data_loop_index==22){
      if (inByte==0xFA)
      {
        Data_loop_index=1;
      } 
      else // if not FA search again
      Data_status=0;
    }
    else{
      readData(inByte);
      Data_loop_index++;
    }
    break;
  }
  
}
void readData(unsigned char inByte){
  switch (Data_loop_index){
    case 1: // 4 degree index
    Data_4deg_index=inByte-0xA0;
//      Serial.print(Data_4deg_index, HEX);  
//      Serial.print(": ");  
    break;
    case 2: // Speed in RPH low byte
    SpeedRPHLowbyte=inByte;
    break;
    case 3: // Speed in RPH high byte
    SpeedRPHhighbyte=inByte;
    SpeedRPH=(SpeedRPHhighbyte<<8)|SpeedRPHLowbyte;
    
//    SpeedControl(DesiredRPM); 
//      Serial.print(SpeedRPHhighbyte, HEX);   
//      Serial.println(SpeedRPHLowbyte, HEX);   
    break;
    default: // others do checksum
        break;
  }  
}

// Very simple speed control
//void SpeedControl ( int RPMinput)
//{
// if (Data_4deg_index%30==0) {  // I only do 3 updat I feel it is good enough for now
//  if (SpeedRPH<RPMinput*60)
//     if (PWM4duty<PWM4dutyMax) PWM4duty++; // limit the max PWM make sure it don't overflow and make LDS stop working
//  if (SpeedRPH>RPMinput*60)
//     if(PWM4duty>PWM4dutyMin) PWM4duty--;  //Have to limit the lowest pwm keep motor running
//  }     
//  analogWrite(MotorPWMPin, PWM4duty ); // update value
//}
