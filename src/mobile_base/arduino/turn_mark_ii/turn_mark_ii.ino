unsigned long lastMilli = 0;

byte LEFT_MOTOR_FORWARD_PIN = 10; //TODO: change me to the right one
byte LEFT_MOTOR_REVERSE_PIN = 9; //TODO: change me to the right one
byte RIGHT_MOTOR_FORWARD_PIN = 6; //TODO: change me to the right one
byte RIGHT_MOTOR_REVERSE_PIN = 5; //TODO: change me to the right one
// Place all MegaMoto Enable jumpers on D8 to turn up with HIGH signal
byte ENABLE_PIN = 8; //TODO: change me to the right one

void setup()
{
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
    analogWrite(LEFT_MOTOR_REVERSE_PIN, 0);
    analogWrite(RIGHT_MOTOR_REVERSE_PIN, 200);
    analogWrite(LEFT_MOTOR_FORWARD_PIN, 200);
    analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
    motorTimer = time;
  }
  lastMilli = time;
  delay(1);
}

