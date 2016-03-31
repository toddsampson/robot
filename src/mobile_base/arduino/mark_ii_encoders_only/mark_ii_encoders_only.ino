 int encoder0PinA = 20;
 int encoder0PinB = 21;
 int encoder0Pos = 0;
 int encoder0PinALast = LOW;
 int n0 = LOW;

 int encoder1PinA = 2;
 int encoder1PinB = 3;
 int encoder1Pos = 0;
 int encoder1PinBLast = LOW;
 int n1 = LOW;

unsigned long lastMilli = 0;

void setup()
{
  Serial.begin(9600);
  pinMode (encoder0PinA,INPUT_PULLUP);
  pinMode (encoder0PinB,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder0PinA), INCRE0, CHANGE);
  
  pinMode (encoder1PinA,INPUT_PULLUP);
  pinMode (encoder1PinB,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), INCRE1, CHANGE);
}

void loop()
{
 unsigned long time = millis();
 static unsigned long motorTimer = 0;
 if(lastMilli - motorTimer > 1000)
  {
    Serial.print(time);
    Serial.print(" \t || 0:");
    Serial.print(encoder0Pos);
    Serial.print(" | 1:");
    Serial.println(encoder1Pos);
    encoder0Pos=0;
    encoder1Pos=0;
    motorTimer = time;
  }
  lastMilli = time;
  delay(1);
}

void INCRE0()
{
   n0 = digitalRead(encoder0PinA);
   if ((encoder0PinALast == LOW) && (n0 == HIGH)) {
     if (digitalRead(encoder0PinB) == LOW) {
       encoder0Pos--;
     } else {
       encoder0Pos++;
     }
   } 
   encoder0PinALast = n0;
}

void INCRE1()
{
   n1 = digitalRead(encoder1PinB);
   if ((encoder1PinBLast == HIGH) && (n1 == HIGH)) {
     if (digitalRead(encoder1PinA) == HIGH) {
       encoder1Pos--;
     } else {
       encoder1Pos++;
     }
   } 
   encoder1PinBLast = n1;
}

