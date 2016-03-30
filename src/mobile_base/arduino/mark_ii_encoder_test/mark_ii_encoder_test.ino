double val=0;
byte  PinA=20;
byte PinB=21;
int ASet;
int BSet;

double right_encval=0;
byte  right_PinA=18; //TODO: change me to the right one
byte right_PinB=19; //TODO: change me to the right one
int right_ASet;
int right_BSet;

byte LEFT_MOTOR_FORWARD_PIN = 6; //TODO: change me to the right one
byte LEFT_MOTOR_REVERSE_PIN = 5; //TODO: change me to the right one
byte RIGHT_MOTOR_FORWARD_PIN = 10; //TODO: change me to the right one
byte RIGHT_MOTOR_REVERSE_PIN = 9; //TODO: change me to the right one
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
  attachInterrupt(digitalPinToInterrupt(PinB), DECRE, CHANGE);

  pinMode(right_PinA, INPUT_PULLUP);
  pinMode(right_PinB, INPUT_PULLUP);
  right_ASet = digitalRead(right_PinA);
  right_BSet = digitalRead(right_PinB);   // read the input pin
  attachInterrupt(digitalPinToInterrupt(right_PinA), right_INCRE, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_PinB), right_DECRE, CHANGE);

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
  analogWrite(RIGHT_MOTOR_REVERSE_PIN, 100);
  analogWrite(LEFT_MOTOR_FORWARD_PIN, 100);
  }
}

void right_INCRE(){
  //Serial.print(" (ri)");
  right_ASet = digitalRead(right_PinA) == HIGH;
  right_BSet = digitalRead(right_PinB) == HIGH;
  right_encval += (right_ASet != right_BSet) ? +1 : -1;
}

void right_DECRE(){
  //Serial.print(" (rd)");
  right_ASet = digitalRead(right_PinA) == HIGH;
  right_BSet = digitalRead(right_PinB) == HIGH;
  right_encval += (right_ASet == right_BSet) ? +1 : -1;
}

void INCRE()
{
  //Serial.print(" (li)");
  ASet = digitalRead(PinA) == HIGH;
  BSet = digitalRead(PinB) == HIGH;
  val += (ASet != BSet) ? +1 : -1;
}

void DECRE()
{
  //Serial.print(" (ld)");
  ASet = digitalRead(PinA) == HIGH;
  BSet = digitalRead(PinB) == HIGH;
  val += (ASet == BSet) ? +1 : -1;
}

