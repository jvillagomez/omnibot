#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <ros.h>
#include <std_msgs/Int64.h>
#include <geometry_msgs/Point.h>

ros::NodeHandle  nh;

Adafruit_MotorShield AFMSbot(0x61); // Rightmost jumper closed
Adafruit_MotorShield AFMStop(0x60); // Default address, no jumpers

Adafruit_StepperMotor *myStepper2 = AFMStop.getStepper(200, 1);
Adafruit_StepperMotor *myStepper3 = AFMStop.getStepper(200, 2);
Adafruit_StepperMotor *myStepper1 = AFMSbot.getStepper(200, 2);

float angularVelocity_x= 0;
float angularVelocity_y= 0;
float angularVelocity_z= 0;

void updateVelocity( const geometry_msgs::Point& velocity_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led


  angularVelocity_x= velocity_msg.x;
  angularVelocity_y= velocity_msg.y;
  angularVelocity_z= velocity_msg.z;
   
  myStepper1->setSpeed(angularVelocity_x);
  myStepper2->setSpeed(angularVelocity_y);
  myStepper3->setSpeed(angularVelocity_z);
}

ros::Subscriber<geometry_msgs::Point> sub("setAngularVelocity", &updateVelocity );


void setup()
{ 
  Serial.begin(9600);        
  Serial.println("Started!");

  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);

  AFMSbot.begin(); // Start the bottom shield
  AFMStop.begin(); // Start the top shield  

//  myStepper1->setSpeed(10);
//  myStepper2->setSpeed(10);
//  myStepper3->setSpeed(10);
}

void loop()
{
  if ( angularVelocity_x > 0.001 ) {
    myStepper1->step(FORWARD, DOUBLE);
  }
  if ( angularVelocity_y > 0.001 ) {
    myStepper2->step(FORWARD, DOUBLE);
  }
  if ( angularVelocity_z > 0.001 ) {
    myStepper3->step(FORWARD, DOUBLE);
  }
  
  nh.spinOnce();
  delay(1);
}

