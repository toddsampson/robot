double val=0;
byte  PinA=18;
byte PinB=19;
int ASet;
int BSet;

unsigned long lastMilli = 0;

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
 unsigned long time = millis();
 static unsigned long motorTimer = 0;
 if(lastMilli - motorTimer > 1000)
  {
    Serial.print(time);
    Serial.print("  \t  l ");
    Serial.print(val);
    Serial.print(" rpm");
    val=0;
    analogWrite(LEFT_MOTOR_REVERSE_PIN, 0);
    analogWrite(RIGHT_MOTOR_REVERSE_PIN, 200);
    analogWrite(LEFT_MOTOR_FORWARD_PIN, 200);
    analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
    motorTimer = time;
  }
  lastMilli = time;
  delay(1);
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

