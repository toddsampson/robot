#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Range.h>
#include <stdio.h>
char str[80];
char str_temp[80];

#define SONAR_PERSONAL_SPACE 15
#define MOTOR_INTERVAL 50
#define MOVEMENT_TIMEOUT 200
#define turnSpeedMin 145
#define turnSpeedMax 200
#define moveSpeedMin 145
#define moveSpeedStart 165
#define moveSpeedMax 255
#define moveBackSpeedMax 255

// sonar pins
byte sl_vcc = 2; //TODO: change me to the right one
byte sl_trig = 3; //TODO: change me to the right one
byte sl_echo = 4; //TODO: change me to the right one
byte sl_gnd = 5; //TODO: change me to the right one
byte sc_vcc = 2; //TODO: change me to the right one
byte sc_trig = 3; //TODO: change me to the right one
byte sc_echo = 4; //TODO: change me to the right one
byte sc_gnd = 5; //TODO: change me to the right one
byte sr_vcc = 2; //TODO: change me to the right one
byte sr_trig = 3; //TODO: change me to the right one
byte sr_echo = 4; //TODO: change me to the right one
byte sr_gnd = 5; //TODO: change me to the right one

// MegaMoto PWM PINS
byte LEFT_MOTOR_FORWARD_PIN = 6; //TODO: change me to the right one
byte LEFT_MOTOR_REVERSE_PIN = 5; //TODO: change me to the right one
byte RIGHT_MOTOR_FORWARD_PIN = 10; //TODO: change me to the right one
byte RIGHT_MOTOR_REVERSE_PIN = 9; //TODO: change me to the right one
// Place all MegaMoto Enable jumpers on D8 to turn up with HIGH signal
byte ENABLE_PIN = 8; //TODO: change me to the right one

//encoders
double left_encval=0;
byte  left_PinA=20; //TODO: change me to the right one
byte left_PinB=21; //TODO: change me to the right one
int left_ASet;
int left_BSet;

double right_encval=0;
byte  right_PinA=18; //TODO: change me to the right one
byte right_PinB=19; //TODO: change me to the right one
int right_ASet;
int right_BSet;

float currX = 0.0;
float currZ = 0.0;
float goalX = 0.0;
float goalZ = 0.0;
boolean running = false;
boolean cb = false;
byte currSpeed = 0;
byte leftHeading = 0; //1 forward, 2 backward
byte rightHeading = 0; //1 forward, 2 backward
byte forwardBlocked = 0; //0 unblocked, 1 blocked
unsigned long lastMssgTime = 0;
float wheelDiameter = 20.32; // In cm (8 in)
byte wheelSeparation = 48.26; // In cm (19 in)
int encoderTicks = 600; // Per rotation
byte gearRatio =  1; //(10:28)
unsigned long lastMilli = 0;
long currCoder0 = 0;
long currCoder1 = 0;
long prevCoder0 = 0;
long prevCoder1 = 0;
long totalCoder0 = 0;
long totalCoder1 = 0;
long totalDiffCnt = 0;
long totalDiffs = 0;

ros::NodeHandle  nh;
std_msgs::String debug_msg;
ros::Publisher Debug ("debug_bot", &debug_msg);
geometry_msgs::Twist twist_msg;
//ros::Publisher Sensorpub ("sensor_debug", &twist_msg);
ros::Publisher Odompub ("odom_debug", &twist_msg);
sensor_msgs::Range sonar_range_msg;
ros::Publisher sl_pub( "sonar_left_depth_frame", &sonar_range_msg);
ros::Publisher sc_pub( "sonar_center_depth_frame", &sonar_range_msg);
ros::Publisher sr_pub( "sonar_right_depth_frame", &sonar_range_msg);
geometry_msgs::Vector3Stamped rpm_msg;
ros::Publisher rpm_pub("rpm", &rpm_msg);

void messageCb(const geometry_msgs::Twist& msg){
  float msgX = msg.linear.x;
  float msgZ = msg.angular.z;
  if(abs(msgX) >= abs(msgZ) && (msgX > 0.1 || msgX < -0.1)){
    goalX = msgX;
    goalZ = 0;
  } else {
    goalX = 0;
    goalZ = msgZ;
  }
  cb = true;
//  debug_msg.data = "RUNNING MSSG CALLBACK";
//  Debug.publish(&debug_msg);
}

int nextSpeed(int minSpeed, int maxSpeed){
  if(currSpeed == 0){
    return minSpeed;
  } else if(currSpeed < maxSpeed){
    return currSpeed + 10;
  }
  return currSpeed;
}

void moveForward(){
  debug_msg.data = "MOVING FORWARD";
  Debug.publish(&debug_msg);
  running = true;
  leftHeading = 1;
  rightHeading = 1;
  currSpeed = nextSpeed(moveSpeedStart, moveSpeedMax);
  analogWrite(RIGHT_MOTOR_REVERSE_PIN, 0);
  analogWrite(LEFT_MOTOR_REVERSE_PIN, 0); 
  analogWrite(RIGHT_MOTOR_FORWARD_PIN, currSpeed);
  analogWrite(LEFT_MOTOR_FORWARD_PIN, currSpeed); 
}

void moveBackward(){
  debug_msg.data = "MOVING BACKWARD";
  Debug.publish(&debug_msg);
  running = true;
  leftHeading = 2;
  rightHeading = 2;
  currSpeed = nextSpeed(moveSpeedStart, moveBackSpeedMax);
  analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
  analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
  analogWrite(RIGHT_MOTOR_REVERSE_PIN, currSpeed);
  analogWrite(LEFT_MOTOR_REVERSE_PIN, currSpeed); 
}

void turnLeft(){
  debug_msg.data = "TURN LEFT";
  Debug.publish(&debug_msg);
  running = true;
  leftHeading = 2;
  rightHeading = 1;
  currSpeed = nextSpeed(turnSpeedMin, turnSpeedMax);
  analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
  analogWrite(LEFT_MOTOR_REVERSE_PIN, 0);
  analogWrite(RIGHT_MOTOR_REVERSE_PIN, currSpeed);
  analogWrite(LEFT_MOTOR_FORWARD_PIN, currSpeed);
}

void turnRight(){
  debug_msg.data = "TURN RIGHT";
  Debug.publish(&debug_msg);
  running = true;
  leftHeading = 1;
  rightHeading = 2;
  currSpeed = nextSpeed(turnSpeedMin, turnSpeedMax);
  analogWrite(RIGHT_MOTOR_REVERSE_PIN, 0);
  analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
  analogWrite(RIGHT_MOTOR_FORWARD_PIN, currSpeed);
  analogWrite(LEFT_MOTOR_REVERSE_PIN, currSpeed); 
}

void stopMovement(){
  debug_msg.data = "MOVEMENT STOPPED";
  Debug.publish(&debug_msg);
  analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
  analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
  analogWrite(RIGHT_MOTOR_REVERSE_PIN, 0);
  analogWrite(LEFT_MOTOR_REVERSE_PIN, 0); 
  running = false;
  currX = 0;
  currZ = 0;
  goalX = 0;
  goalZ = 0;
  currSpeed = 0;
  leftHeading = 0;
  rightHeading = 0;
}


boolean movingForward(){
  if(leftHeading == 1 && rightHeading == 1){
    return true;
  }
  return false;
}

boolean movingBackward(){
  if(leftHeading == 2 && rightHeading == 2){
    return true;
  }
  return false;
}

boolean turningLeft(){
  if(leftHeading == 2 && rightHeading == 1){
    return true;
  }
  return false;
}

boolean turningRight(){
  if(leftHeading == 1 && rightHeading == 2){
    return true;
  }
  return false;
}

//flipped the +1 and -1
void left_INCRE(){
//  debug_msg.data = "LEFT INCRE";
//  Debug.publish(&debug_msg);
  left_ASet = digitalRead(left_PinA) == HIGH;
  left_encval += (left_ASet != left_BSet) ? -1 : +1;
}

void left_DECRE(){
//  dtostrf(left_encval, 4, 2, str_temp);
//  sprintf(str, "left decre: %s",str_temp);
//  debug_msg.data = str;
//  Debug.publish(&debug_msg);

  left_BSet = digitalRead(left_PinB) == HIGH;
  left_encval += (left_ASet == left_BSet) ? -1 : +1;
}

void right_INCRE(){
//  debug_msg.data = "RIGHT INCRE";
//  Debug.publish(&debug_msg);
  right_ASet = digitalRead(right_PinA) == HIGH;
  right_encval += (right_ASet != right_BSet) ? -1 : +1;
}

void right_DECRE(){
//  debug_msg.data = "RIGHT DECRE";
//  Debug.publish(&debug_msg);
  right_BSet = digitalRead(right_PinB) == HIGH;
  right_encval += (right_ASet == right_BSet) ? -1 : +1;
}

boolean sonarBlocked(int val){
  if(val > 0 && val < SONAR_PERSONAL_SPACE){
    return true;
  }
  return false;
}

boolean sensorBlocked(int sLeft, int sRight, int sCenter){
  if(sonarBlocked(sLeft) || sonarBlocked(sRight) || sonarBlocked(sCenter)){
    return true;
  }
  return false;
}

//void debugSensors(int sLeft, int sRight, int sCenter){
//  twist_msg.linear.x = sLeft;
//  twist_msg.linear.y = sCenter;
//  twist_msg.linear.z = sRight;
//  twist_msg.angular.y = forwardBlocked;
//  Sensorpub.publish(&twist_msg);
//}

void publishSonar(float sLeft, float sRight, float sCenter){
  char sl_frameid[] = "/sonar_left_depth_frame";
  char sr_frameid[] = "/sonar_right_depth_frame";
  char sc_frameid[] = "/sonar_center_depth_frame";

  sLeft = sLeft / 100; //because we want to send the value in meters
  sonar_range_msg.header.frame_id =  sl_frameid;
  sonar_range_msg.range = sLeft;
  sonar_range_msg.header.stamp = nh.now();
  sl_pub.publish(&sonar_range_msg);

  sRight = sRight / 100; //because we want to send the value in meters
  sonar_range_msg.header.frame_id =  sr_frameid;
  sonar_range_msg.range = sRight;
  sonar_range_msg.header.stamp = nh.now();
  sr_pub.publish(&sonar_range_msg);

  sCenter = sCenter / 100; //because we want to send the value in meters
  sonar_range_msg.header.frame_id =  sc_frameid;
  sonar_range_msg.range = sCenter;
  sonar_range_msg.header.stamp = nh.now();
  sc_pub.publish(&sonar_range_msg);
  //nh.spinOnce();
}

void checkForBlocks(float sLeft, float sRight, float sCenter){
  bool blocked = sensorBlocked(sLeft, sRight, sCenter);
  if(blocked){
//    debug_msg.data = "BLOCKING FORWARD MOTION";
//    Debug.publish(&debug_msg);
    forwardBlocked = 1;    
  } else {
//      debug_msg.data = "FORWARD MOTION UNBLOCKED";
//      Debug.publish(&debug_msg); 
    forwardBlocked = 0;
  }

  if(blocked && goalX > 0.1){
    goalX = 0;
    goalZ = 0;
    debug_msg.data = "SHOULD STOP FORWARD MOTION by setting goal velocities to 0";
    Debug.publish(&debug_msg); 
  }
}

long sonarDistance(int vcc, int trig, int echo){
  digitalWrite(vcc, HIGH);
  long duration;
  pinMode(trig, OUTPUT);
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(5);
  digitalWrite(trig, LOW);
  pinMode(echo,INPUT);
  duration = pulseIn(echo, HIGH);
  return microsecondsToCentimeters(duration);
}

void checkSensors(){
  float sLeft = sonarDistance(sl_vcc, sl_trig, sl_echo);
  float sCenter = sonarDistance(sc_vcc, sc_trig, sc_echo);
  float sRight = sonarDistance(sr_vcc, sr_trig, sr_echo);

  checkForBlocks(sLeft, sRight, sCenter);
  publishSonar(sLeft, sRight, sCenter);
  //debugSensors(sLeft, sRight, sCenter);
}


long microsecondsToCentimeters(long microseconds)
{
// The speed of sound is 340 m/s or 29 microseconds per centimeter.
// The ping travels out and back, so to find the distance of the
// object we take half of the distance travelled.
return microseconds / 29 / 2;
}

void controlMotors(){
  debug_msg.data = "CONTROL MOTORS";
  Debug.publish(&debug_msg); 
  if(goalX != currX || goalZ != currZ || goalX > 0.1 || goalX < -0.1 || goalZ > 0.1 || goalZ < -0.1){
    if(cb == true){
      lastMssgTime = millis();
      cb = false;
    }
    currX = goalX;  // later we will slowly ramp curr up towards goal
    currZ = goalZ;  // and use an accel method to determine speed to set
    if(currX > 0.1 || currX < -0.1 || currZ > 0.1 || currZ < -0.1){
//      debug_msg.data = "NEW ACTION STARTING";
//      Debug.publish(&debug_msg);
      if(currX > 0.1){
        if(forwardBlocked == 0){
          moveForward();
        }
      } else if(currX < -0.1){
        moveBackward();
      } else if(currZ > 0.1){
        turnLeft();
      } else if(currZ < -0.1){
        turnRight();
      }
    } else {
      stopMovement();
    }
  }
//  debugSensors(millis()/1000, lastMssgTime/1000,(millis() - lastMssgTime)/1000);  
  if((millis() - lastMssgTime) > MOVEMENT_TIMEOUT){
//    debug_msg.data = "STOPPING MOVEMENT DUE TO LASTMSSGTIME TIMEOUT";
//    Debug.publish(&debug_msg);    
    goalX = 0;
    goalZ = 0;
  }  
}

void debugOdom(double totalCoder0, double totalCoder1, long currCoder0, long currCoder1, long rpm0, long rpm1){
  twist_msg.linear.x = totalCoder0;
  twist_msg.linear.y = totalCoder1;
  twist_msg.linear.z = currCoder0;
  twist_msg.angular.x = currCoder1;
  twist_msg.angular.y = rpm0;
  twist_msg.angular.z = rpm1;
  Odompub.publish(&twist_msg);
}

void publishOdom(double vel_lx, double vel_az, unsigned long time){
  rpm_msg.header.stamp = nh.now();
  rpm_msg.vector.x = vel_lx;
  rpm_msg.vector.y = vel_az;
  rpm_msg.vector.z = double(time)/1000;
  rpm_pub.publish(&rpm_msg);
  nh.spinOnce(); 
}

void handleOdometry(unsigned long time){
  debug_msg.data = "HANDLE ODOM";
  Debug.publish(&debug_msg); 
  double vel_lx = 0; // odom linear x velocity
  double vel_az = 0; // odom angular z velocity
  totalCoder0 = left_encval;  // this method of holding the encoder value
  totalCoder1 = right_encval;  // prevents us from losing any ticks
  currCoder0 = totalCoder0 - prevCoder0;
  currCoder1 = totalCoder1 - prevCoder1;
  prevCoder0 = totalCoder0;
  prevCoder1 = totalCoder1;
  double elapsed = time/(double)1000;
  double tickRatio0 = (double)currCoder0/encoderTicks;
  double tickRatio1 = (double)currCoder1/encoderTicks;
  vel_lx = double((currCoder0)*60*1000)/double(time*encoderTicks*gearRatio);
  vel_az = double((currCoder1)*60*1000)/double(time*encoderTicks*gearRatio);

  debugOdom(tickRatio0, tickRatio1, currCoder0, currCoder1, vel_lx, vel_az);
  publishOdom(vel_lx, vel_az, time);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", messageCb);

void setup(){
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(rpm_pub);
  nh.advertise(Debug);
  nh.advertise(Odompub);
//  nh.advertise(Sensorpub);

  //encoders
  pinMode(left_PinA, INPUT_PULLUP);
  pinMode(left_PinB, INPUT_PULLUP);
  left_ASet = digitalRead(left_PinA);
  left_BSet = digitalRead(left_PinB);   // read the input pin
  attachInterrupt(digitalPinToInterrupt(left_PinA), left_INCRE, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_PinB), left_DECRE, CHANGE);
  
  pinMode(right_PinA, INPUT_PULLUP);
  pinMode(right_PinB, INPUT_PULLUP);
  right_ASet = digitalRead(right_PinA);
  right_BSet = digitalRead(right_PinB);   // read the input pin
  attachInterrupt(digitalPinToInterrupt(right_PinA), right_INCRE, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_PinB), right_DECRE, CHANGE);

  //motors
  //Initialize Motor shields for Arduino (MegaMoto shields)
  //pinMode(CH2_PIN, INPUT);
  //pinMode(CH4_PIN, INPUT);
  //ENABLE MOTOR SHIELDS
  pinMode(ENABLE_PIN,OUTPUT);
  digitalWrite(ENABLE_PIN,HIGH);
  //STOP ALL MOTORS
  analogWrite(RIGHT_MOTOR_FORWARD_PIN,0);
  analogWrite(RIGHT_MOTOR_REVERSE_PIN,0);
  analogWrite(LEFT_MOTOR_FORWARD_PIN,0);
  analogWrite(LEFT_MOTOR_REVERSE_PIN,0);

  //sonar
  nh.advertise(sl_pub);
  nh.advertise(sr_pub);
  nh.advertise(sc_pub);
  sonar_range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  sonar_range_msg.field_of_view = 0.7;
  sonar_range_msg.min_range = 0.02;
  sonar_range_msg.max_range = 3; 
//  pinMode (sl_vcc,OUTPUT);
//  pinMode (sl_gnd,OUTPUT);
//  pinMode (sc_vcc,OUTPUT);
//  pinMode (sc_gnd,OUTPUT);
//  pinMode (sr_vcc,OUTPUT);
//  pinMode (sr_gnd,OUTPUT);
}

void loop(){
  debug_msg.data = "LOOP";
  Debug.publish(&debug_msg); 
  static unsigned long motorTimer = 0;
  unsigned long time = millis();
  nh.spinOnce();

  if(lastMilli - motorTimer > MOTOR_INTERVAL){
    debug_msg.data = "LOOP INTERVAL";
    Debug.publish(&debug_msg); 
    handleOdometry(time-motorTimer); //TODO turn odom back on
    //checkSensors();    //TODO turn sensors back on
    controlMotors();
    motorTimer = time;
  }

  lastMilli = time;
  delay(1);
}

