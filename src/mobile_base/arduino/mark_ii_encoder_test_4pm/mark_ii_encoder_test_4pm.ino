double val=0;
byte  PinA=2;
byte PinB=3;

double right_encval=0;
byte  right_PinA=20; //TODO: change me to the right one
byte right_PinB=21; //TODO: change me to the right one

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
  attachInterrupt(digitalPinToInterrupt(PinA), INCRE, CHANGE);

  pinMode(right_PinA, INPUT_PULLUP);
  pinMode(right_PinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(right_PinA), right_INCRE, CHANGE);

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

    Serial.print("  \t  r ");
    Serial.print(right_encval);
    Serial.println(" rpm");
    val=0;
    right_encval = 0;
    analogWrite(LEFT_MOTOR_REVERSE_PIN, 0);
    analogWrite(RIGHT_MOTOR_REVERSE_PIN, 200);
    analogWrite(LEFT_MOTOR_FORWARD_PIN, 200);
    analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
    motorTimer = time;
  }
  lastMilli = time;
  delay(1);
}

void right_INCRE(){
  if(digitalRead(right_PinA) == digitalRead(right_PinB)){
    right_encval++;
  } else {
    right_encval--;
  }
}

void INCRE(){
  if(digitalRead(PinA) != digitalRead(PinB)){
    val++;  
  } else {
    val--;
  }
}

