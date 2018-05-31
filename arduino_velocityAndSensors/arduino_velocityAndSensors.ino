#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <ros.h>
#include <std_msgs/Int64.h>
#include <geometry_msgs/Point.h>
// ==================================================
#include <CurieIMU.h>
#include <MadgwickAHRS.h>

#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>

ros::NodeHandle  nh;

Adafruit_MotorShield AFMSbot(0x61); // Rightmost jumper closed
Adafruit_MotorShield AFMStop(0x60); // Default address, no jumpers

Adafruit_StepperMotor *stepMotor_2 = AFMStop.getStepper(200, 1);
Adafruit_StepperMotor *stepMotor_3 = AFMStop.getStepper(200, 2);
Adafruit_StepperMotor *stepMotor_1 = AFMSbot.getStepper(200, 2);

float stepMotor1_vel= 0;
float stepMotor2_vel= 0;
float stepMotor3_vel= 0;

// ==================================================
// START orientation
// ==================================================
geometry_msgs::Vector3 orientation;
geometry_msgs::Vector3 linearAccel;
geometry_msgs::Vector3 angularAccel;

ros::Publisher linearAccel_topic("linearAccel_topic", &linearAccel);
ros::Publisher angularAccel_topic("angularAccel_topic", &angularAccel);
ros::Publisher orientation_topic("orientation_topic", &orientation);

Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;
// ==================================================
// END orientation
// ==================================================

void updateMotorVelocities( const geometry_msgs::Point& velocity_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led

  stepMotor1_vel = velocity_msg.x;
  stepMotor2_vel = velocity_msg.y;
  stepMotor3_vel = velocity_msg.z;

  stepMotor_1->setSpeed(stepMotor1_vel);
  stepMotor_2->setSpeed(stepMotor2_vel);
  stepMotor_3->setSpeed(stepMotor3_vel);
}

ros::Subscriber<geometry_msgs::Point> sub("setMotorVelocities_topic", &updateMotorVelocities );


void setup()
{
  Serial.begin(115200);
  Serial.println("Started!");

  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
  // ==================================================
  // START orientation
  // ==================================================
  nh.advertise(orientation_topic);
  nh.advertise(linearAccel_topic);
  nh.advertise(angularAccel_topic);

  // start the IMU and filter
  CurieIMU.begin();
  CurieIMU.setGyroRate(25);
  CurieIMU.setAccelerometerRate(25);
  filter.begin(25);

  // Set the accelerometer range to 2G
  CurieIMU.setAccelerometerRange(2);
  // Set the gyroscope range to 250 degrees/second
  CurieIMU.setGyroRange(250);

  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 25;
  microsPrevious = micros();
  // ==================================================
  // END orientation
  // ==================================================
  AFMSbot.begin(); // Start the bottom shield
  AFMStop.begin(); // Start the top shield
}

void setMotor1Velocity(float motorVel)
{
  if ( abs(motorVel) > 0.01 ) {
    if(motorVel < 0) {
        stepMotor_1->step(BACKWARD, DOUBLE);
    }
    stepMotor_1->step(FORWARD, DOUBLE);
  }
}
void setMotor2Velocity(float motorVel)
{
  if ( abs(motorVel) > 0.01 ) {
    if(motorVel < 0) {
        stepMotor_2->step(BACKWARD, DOUBLE);
    }
    stepMotor_2->step(FORWARD, DOUBLE);
  }
}
void setMotor3Velocity(float motorVel)
{
  if ( abs(motorVel) > 0.01 ) {
    if(motorVel < 0) {
        stepMotor_3->step(BACKWARD, DOUBLE);
    }
    stepMotor_3->step(FORWARD, DOUBLE);
  }
}

void loop()
{
  setMotor1Velocity(stepMotor1_vel);
  setMotor3Velocity(stepMotor2_vel);
  setMotor2Velocity(stepMotor3_vel);
  // if ( stepMotor1_vel > 0.001 ) {
  //   stepMotor_1->step(FORWARD, DOUBLE);
  // }
  // if ( stepMotor2_vel > 0.001 ) {
  //   stepMotor_2->step(FORWARD, DOUBLE);
  // }
  // if ( stepMotor3_vel > 0.001 ) {
  //   stepMotor_3->step(FORWARD, DOUBLE);
  // }

  // nh.spinOnce();
  // delay(1);
  // ==================================================
  // START orientation
  // ==================================================
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, heading;
  unsigned long microsNow;

  // check if it's time to read data and update the filter
  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {

    // read raw data from CurieIMU
    CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

    // convert from raw data to gravity and degrees/second units
    ax = convertRawAcceleration(aix);
    ay = convertRawAcceleration(aiy);
    az = convertRawAcceleration(aiz);
    gx = convertRawGyro(gix);
    gy = convertRawGyro(giy);
    gz = convertRawGyro(giz);

    // update the filter, which computes orientation
    filter.updateIMU(gx, gy, gz, ax, ay, az);

    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();

    linearAccel.x = ax;
    linearAccel.y = ay;
    linearAccel.z = az;
    angularAccel.x = gx;
    angularAccel.y = gy;
    angularAccel.z = gz;

    orientation.x = roll;
    orientation.y = pitch;
    orientation.z = heading;

    linearAccel_topic.publish( &linearAccel );
    angularAccel_topic.publish( &angularAccel );
    orientation_topic.publish( &orientation );

    nh.spinOnce();

    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
  // ==================================================
  // END orientation
  // ==================================================
}

// ==================================================
// START orientation
// ==================================================
float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767

  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767

  float g = (gRaw * 250.0) / 32768.0;
  return g;
}
// ==================================================
// END orientation
// ==================================================
