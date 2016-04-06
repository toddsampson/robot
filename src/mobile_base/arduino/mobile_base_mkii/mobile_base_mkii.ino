#include <Arduino.h>
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Range.h>
#include <stdio.h>

#define SL_VCC 2 // sonar pins
#define SL_TRIG 3 //TODO: change me to the right ones
#define SL_ECHO 4
#define SL_GND 5
#define SC_VCC 2
#define SC_TRIG 3
#define SC_ECHO 4
#define SC_GND 5
#define SR_VCC 2
#define SR_TRIG 3
#define SR_ECHO 4
#define SR_GND 5
#define LEFT_PIN_A 20 //encoder pins
#define LEFT_PIN_B 21
#define  LEFT_MOTOR_FORWARD_PIN 10 // motor pins
#define  LEFT_MOTOR_REVERSE_PIN 9
#define  RIGHT_MOTOR_FORWARD_PIN 6
#define  RIGHT_MOTOR_REVERSE_PIN 5
#define  ENABLE_PIN 8

#define SONAR_PERSONAL_SPACE 15
#define MOTOR_INTERVAL 50 // running motors and reporting rpm at 20 hz
#define MOVEMENT_TIMEOUT 255
#define TURN_SPEED_MIN 145
#define TURN_SPEED_MAX 200
#define MOVE_SPEED_MIN 165
#define MOVE_SPEED_MAX 255

float wheelDiameter = 20.32; // In cm (8 in)
byte wheelSeparation = 48.26; // In cm (19 in)
int encoderTicks = 1680; // Per rotation
byte gearRatio =  1; //(10:28) // we are using one because we set out encoder to 1680 instead of 600
volatile long left_encval=0;
float currX = 0.0;
float currZ = 0.0;
float goalX = 0.0;
float goalZ = 0.0;
boolean cb = false;
boolean negateOtherEnc = false;
byte currSpeed = 0;
byte forwardBlocked = 0; //0 unblocked, 1 blocked
unsigned long lastMssgTime = 0;
long prevCoder0 = 0;

ros::NodeHandle  nh;
std_msgs::String debugMsg; //should be off in production - only used by debug_bot
geometry_msgs::Twist twistMsg;
geometry_msgs::Vector3Stamped RPMMsg;
sensor_msgs::Range sonarRangeMsg;
ros::Publisher RPMPub("rpm", &RPMMsg);
ros::Publisher SLPub( "sonar_left_depth_frame", &sonarRangeMsg);
ros::Publisher SCPub( "sonar_center_depth_frame", &sonarRangeMsg);
ros::Publisher SRPub( "sonar_right_depth_frame", &sonarRangeMsg);
ros::Publisher DebugPub ("debug_bot", &debugMsg); //should be off in production
//ros::Publisher OdomPub ("odom_debug", &twistMsg); //should be off in production
//ros::Publisher Sensorpub ("sensor_debug", &twistMsg); //should be off in production

/* --------------------------------- */
/* SENSORS ------------------------- */
/* --------------------------------- */
void checkSensors(){
  float sLeft = sonarDistance(SL_VCC, SL_TRIG, SL_ECHO);
  float sCenter = sonarDistance(SC_VCC, SC_TRIG, SC_ECHO);
  float sRight = sonarDistance(SR_VCC, SR_TRIG, SR_ECHO);
  checkForBlocks(sLeft, sCenter, sRight);
  publishSonar(sLeft, sCenter, sRight);
  //debugSensors(sLeft, sCenter, sRight);
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

void checkForBlocks(float sLeft, float sCenter, float sRight){
  bool blocked = sensorBlocked(sLeft, sCenter, sRight);
  if(blocked) forwardBlocked = 1;    
  else forwardBlocked = 0;
  if(blocked && goalX > 0.1){
    goalX = 0;
    goalZ = 0;
    debugMsg.data = "SHOULD STOP FORWARD MOTION by setting goal velocities to 0";
    DebugPub.publish(&debugMsg); 
  }
}

boolean sensorBlocked(int sLeft, int sCenter, int sRight){
  if(sonarBlocked(sLeft) || sonarBlocked(sCenter) || sonarBlocked(sRight)) return true;
  else return false;
}

boolean sonarBlocked(int val){
  if(val > 0 && val < SONAR_PERSONAL_SPACE) return true;
  else return false;
}

void publishSonar(float sLeft, float sCenter, float sRight){
  char sl_frameid[] = "/sonar_left_depth_frame";
  sLeft = sLeft / 100; //because we want to send the value in meters
  sonarRangeMsg.header.frame_id =  sl_frameid;
  sonarRangeMsg.range = sLeft;
  sonarRangeMsg.header.stamp = nh.now();
  SLPub.publish(&sonarRangeMsg);

  char sc_frameid[] = "/sonar_center_depth_frame";
  sCenter = sCenter / 100; //because we want to send the value in meters
  sonarRangeMsg.header.frame_id =  sc_frameid;
  sonarRangeMsg.range = sCenter;
  sonarRangeMsg.header.stamp = nh.now();
  SCPub.publish(&sonarRangeMsg);

  char sr_frameid[] = "/sonar_right_depth_frame";
  sRight = sRight / 100; //because we want to send the value in meters
  sonarRangeMsg.header.frame_id =  sr_frameid;
  sonarRangeMsg.range = sRight;
  sonarRangeMsg.header.stamp = nh.now();
  SRPub.publish(&sonarRangeMsg);
}

long microsecondsToCentimeters(long microseconds){
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}

//void debugSensors(int sLeft, int sCenter, int sRight){
//  twistMsg.linear.x = sLeft;
//  twistMsg.linear.y = sCenter;
//  twistMsg.linear.z = sRight;
//  twistMsg.angular.y = forwardBlocked;
//  Sensorpub.publish(&twistMsg);
//}

/* --------------------------------- */
/* MOTORS -------------------------- */
/* --------------------------------- */
// callback for receiving a cmd_vel command
// we only expect a value on x or z
// and so we only set one of x or z
// and thus our robot either moves or turns
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
}

ros::Subscriber<geometry_msgs::Twist> cmd_ctrl("cmd_vel", messageCb);

void controlMotors(){
  if(goalX != currX || goalZ != currZ || goalX > 0.1 || goalX < -0.1 || goalZ > 0.1 || goalZ < -0.1){
    if(cb == true){
      lastMssgTime = millis();
      cb = false;
    }
    currX = goalX;
    currZ = goalZ;
    if(currX > 0.1 || currX < -0.1 || currZ > 0.1 || currZ < -0.1){
      // debugMsg.data = "NEW ACTION STARTING";
      // DebugPub.publish(&debugMsg);
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
  if((millis() - lastMssgTime) > MOVEMENT_TIMEOUT){
    // debugMsg.data = "STOPPING MOVEMENT DUE TO LASTMSSGTIME TIMEOUT";
    // DebugPub.publish(&debugMsg);    
    goalX = 0;
    goalZ = 0;
  }  
}

void moveForward(){
  negateOtherEnc = false;
  currSpeed = nextSpeed(MOVE_SPEED_MIN, MOVE_SPEED_MAX);
  analogWrite(RIGHT_MOTOR_REVERSE_PIN, 0);
  analogWrite(LEFT_MOTOR_REVERSE_PIN, 0); 
  analogWrite(RIGHT_MOTOR_FORWARD_PIN, currSpeed);
  analogWrite(LEFT_MOTOR_FORWARD_PIN, currSpeed); 
}

void moveBackward(){
  negateOtherEnc = false;
  currSpeed = nextSpeed(MOVE_SPEED_MIN, MOVE_SPEED_MAX);
  analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
  analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
  analogWrite(RIGHT_MOTOR_REVERSE_PIN, currSpeed);
  analogWrite(LEFT_MOTOR_REVERSE_PIN, currSpeed); 
}

void turnLeft(){
  negateOtherEnc = true;
  currSpeed = nextSpeed(TURN_SPEED_MIN, TURN_SPEED_MAX);
  analogWrite(RIGHT_MOTOR_REVERSE_PIN, 0);
  analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
  analogWrite(RIGHT_MOTOR_FORWARD_PIN, currSpeed);
  analogWrite(LEFT_MOTOR_REVERSE_PIN, currSpeed);
}

void turnRight(){
  negateOtherEnc = true;
  currSpeed = nextSpeed(TURN_SPEED_MIN, TURN_SPEED_MAX);
  analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
  analogWrite(LEFT_MOTOR_REVERSE_PIN, 0);
  analogWrite(RIGHT_MOTOR_REVERSE_PIN, currSpeed);
  analogWrite(LEFT_MOTOR_FORWARD_PIN, currSpeed); 
}

void stopMovement(){
  debugMsg.data = "MOVEMENT STOPPED";
  DebugPub.publish(&debugMsg);
  analogWrite(RIGHT_MOTOR_FORWARD_PIN, 0);
  analogWrite(LEFT_MOTOR_FORWARD_PIN, 0);
  analogWrite(RIGHT_MOTOR_REVERSE_PIN, 0);
  analogWrite(LEFT_MOTOR_REVERSE_PIN, 0); 
  currX = 0;
  currZ = 0;
  goalX = 0;
  goalZ = 0;
  currSpeed = 0;
}

int nextSpeed(int minSpeed, int maxSpeed){
  if(currSpeed == 0) return minSpeed;
  else if(currSpeed < maxSpeed) return currSpeed + 10;
  else return currSpeed;
}

/* --------------------------------- */
/* PUBLISHING RPM ------------------ */
/* --------------------------------- */
// isr (or callback) for the encoder, on called on rising pinA
void leftEncCb(){
  if(digitalRead(LEFT_PIN_A) != digitalRead(LEFT_PIN_B))left_encval++;
  else left_encval--;
}

void handleOdometry(unsigned long time){
  double vel_l = 0;
  double vel_r = 0;
  long totalCoder0 = left_encval;
  long currCoder0 = totalCoder0 - prevCoder0;
  prevCoder0 = totalCoder0;
  vel_l = double((currCoder0)*60*1000)/double(time*encoderTicks*gearRatio);
  vel_r = getOtherEncVal(vel_l);
  // debugOdom(vel_l, vel_r, totalCoder0, prevCoder0, currCoder0, 0);
  publishOdom(vel_l, vel_r, time);
}

void publishOdom(double vel_l, double vel_r, unsigned long time){
  RPMMsg.header.stamp = nh.now();
  RPMMsg.vector.x = vel_l;
  RPMMsg.vector.y = vel_r;
  RPMMsg.vector.z = double(time)/1000;
  RPMPub.publish(&RPMMsg);
  nh.spinOnce(); 
}

// we are using this method to assume perfect wheel sync
// and write out a matching positive or negative rpm value for the other wheel
// based on whether we are turning or driving straight
double getOtherEncVal(double left_val){
  if(negateOtherEnc == true) return -left_val;
  else return left_val;
}

//void debugOdom(double a, double b, long c, long d, long e, long f){
//  twistMsg.linear.x = a;
//  twistMsg.linear.y = b;
//  twistMsg.linear.z = c;
//  twistMsg.angular.x = d;
//  twistMsg.angular.y = e;
//  twistMsg.angular.z = f;
//  OdomPub.publish(&twistMsg);
//}

/* --------------------------------- */
/* MAIN PROGRAM -------------------- */
/* --------------------------------- */
void setup(){
  nh.initNode(); // start ros
  nh.advertise(DebugPub); // general debug messages

  //encoders
  nh.advertise(RPMPub);
  // nh.advertise(OdomPub); // odom debug messages
  pinMode(LEFT_PIN_A, INPUT_PULLUP);
  pinMode(LEFT_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_PIN_A), leftEncCb, RISING);

  //motors
  nh.subscribe(cmd_ctrl); // cmd vel messages
  pinMode(ENABLE_PIN,OUTPUT); // ENABLE MOTOR SHIELDS
  digitalWrite(ENABLE_PIN,HIGH);
  analogWrite(RIGHT_MOTOR_FORWARD_PIN,0); // STOP ALL MOTORS
  analogWrite(RIGHT_MOTOR_REVERSE_PIN,0);
  analogWrite(LEFT_MOTOR_FORWARD_PIN,0);
  analogWrite(LEFT_MOTOR_REVERSE_PIN,0);

  //sonar
  // nh.advertise(Sensorpub); // sensor debugging messages
  nh.advertise(SLPub); // sensor left status messages
  nh.advertise(SCPub); // sensor center status messages
  nh.advertise(SRPub); // sensor right status messages 
  sonarRangeMsg.radiation_type = sensor_msgs::Range::ULTRASOUND;
  sonarRangeMsg.field_of_view = 0.7;
  sonarRangeMsg.min_range = 0.02;
  sonarRangeMsg.max_range = 3; 
//  pinMode (SL_VCC,OUTPUT);
//  pinMode (SL_GND,OUTPUT);
//  pinMode (SC_VCC,OUTPUT);
//  pinMode (SC_GND,OUTPUT);
//  pinMode (SR_VCC,OUTPUT);
//  pinMode (SR_GND,OUTPUT);
}

void loop(){
  static unsigned long motorTimer = 0;
  unsigned long time = millis();
  nh.spinOnce();
  if(time - motorTimer > MOTOR_INTERVAL){
    handleOdometry(time-motorTimer);
    //checkSensors();    //TODO turn sensors back on
    controlMotors();
    motorTimer = time;
  }
}
