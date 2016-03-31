volatile double val=0;
byte  PinA=20;
byte PinB=21;
volatile int ASet;
volatile int BSet;
volatile int AWas = LOW;
volatile int AIs;
volatile int BWas = LOW;
volatile int BIs;

volatile double right_encval=0;
byte  right_PinA=2; //TODO: change me to the right one
byte right_PinB=3; //TODO: change me to the right one
volatile int right_ASet;
volatile int right_BSet;
volatile int right_AWas = LOW;
volatile int right_AIs;
volatile int right_BWas = LOW;
volatile int right_BIs;

byte LEFT_MOTOR_FORWARD_PIN = 10; //TODO: change me to the right one
byte LEFT_MOTOR_REVERSE_PIN = 9; //TODO: change me to the right one
byte RIGHT_MOTOR_FORWARD_PIN = 6; //TODO: change me to the right one
byte RIGHT_MOTOR_REVERSE_PIN = 5; //TODO: change me to the right one
// Place all MegaMoto Enable jumpers on D8 to turn up with HIGH signal
byte ENABLE_PIN = 8; //TODO: change me to the right one

void setup()
{
  Serial.begin(9600);
  pinMode(PinA, INPUT_PULLUP);
  pinMode(PinB, INPUT_PULLUP);
  ASet = digitalRead(PinA);
  BSet = digitalRead(PinB);   // read the input pin
  attachInterrupt(digitalPinToInterrupt(PinA), INCRE, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PinB), INCRE, CHANGE);

  pinMode(right_PinA, INPUT_PULLUP);
  pinMode(right_PinB, INPUT_PULLUP);
  right_ASet = digitalRead(right_PinA);
  right_BSet = digitalRead(right_PinB);   // read the input pin
  attachInterrupt(digitalPinToInterrupt(right_PinA), right_INCRE, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_PinB), right_INCRE, CHANGE);

  pinMode(ENABLE_PIN,OUTPUT);
  digitalWrite(ENABLE_PIN,HIGH);
  //STOP ALL MOTORS
  analogWrite(RIGHT_MOTOR_FORWARD_PIN,0);
  analogWrite(RIGHT_MOTOR_REVERSE_PIN,0);
  analogWrite(LEFT_MOTOR_FORWARD_PIN,0);
  analogWrite(LEFT_MOTOR_REVERSE_PIN,0);
}

void loop()
{
  int time=millis()/1000; 
 if(time % 10==0)
  {
    Serial.print(time);
    Serial.print("  \t  l ");
    Serial.print(val);
    Serial.print(" rpm");

    Serial.print("  \t  r ");
    Serial.print(right_encval);
    Serial.println(" rpm");
    val=0;
    right_encval = 0;
    delay(1000);
  analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
  analogWrite(LEFT_MOTOR_REVERSE_PIN, 0);
  analogWrite(RIGHT_MOTOR_REVERSE_PIN, 0);
  analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
  }
}

void right_INCRE(){
  right_AIs = digitalRead(right_PinA);
  if(right_AWas == LOW && right_AIs == HIGH){
    right_encval += (digitalRead(right_PinB) == LOW) ? +1 : -1;
  }
  right_AWas = right_AIs;
}

//void right_DECRE(){
//  right_BIs = digitalRead(right_PinB);
//  if(right_BWas == LOW && right_BIs == HIGH){
//    right_ASet = digitalRead(right_PinA) == HIGH;
//    right_BSet = right_BIs == HIGH;
//    right_encval += (right_ASet == right_BSet) ? +1 : -1;
//  }
//  BWas = BIs;
//}

void INCRE()
{
  AIs = digitalRead(PinA);
  if(AWas == LOW && AIs == HIGH){
    val += (digitalRead(PinB) == HIGH) ? +1 : -1;
  }
  AWas = AIs;
}
//
//void DECRE()
//{
//  BIs = digitalRead(PinB);
//  if(BWas == LOW && BIs == HIGH){
//    ASet = digitalRead(PinA) == HIGH;
//    BSet = BIs == HIGH;
//    val += (ASet == BSet) ? +1 : -1;
//  }
//  BWas = BIs;
//}

