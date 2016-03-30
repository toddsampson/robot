double val=0;
byte  PinA=8;
byte PinB=7;
int ASet;
int BSet;

void setup()
{
  Serial.begin(9600);
  pinMode(PinA, INPUT_PULLUP);
  pinMode(PinB, INPUT_PULLUP);
  

  ASet = digitalRead(PinA);
  BSet = digitalRead(PinB);   // read the input pin
 
  attachInterrupt(digitalPinToInterrupt(PinA), INCRE, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PinB), DECRE, CHANGE);
 
  //analogWrite(8, 100);        // for Motor Driving purpose only
}

void loop()
{
  int time=millis()/1000; 
 if(time % 2==0)
  {
    Serial.print(time);
    Serial.print("  \t  ");
    Serial.print(val*6/200);
    Serial.println(" rpm");
    val=0;
    delay(200);
  }
}

void INCRE()
{
  ASet = digitalRead(PinA) == HIGH;
  val += (ASet != BSet) ? +1 : -1;
}

void DECRE()
{
  BSet = digitalRead(PinB) == HIGH;
  val += (ASet == BSet) ? +1 : -1;
}

