#include <ros.h>
#include <CurieIMU.h>
#include <MadgwickAHRS.h>

#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>


ros::NodeHandle  nh;

geometry_msgs::Vector3 orientation;
geometry_msgs::Vector3 linearAccel;
geometry_msgs::Vector3 angularAccel;

ros::Publisher orientation_topic("orientation_topic", &orientation);
ros::Publisher linearAccel_topic("linearAccel_topic", &linearAccel);
ros::Publisher angularAccel_topic("angularAccel_topic", &angularAccel);

Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

void setup() {
  
  Serial.begin(9600);

  nh.initNode();
  nh.advertise(orientation_topic);

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
}

void loop() {
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

//    linearAccel.x = ax;
//    linearAccel.y = ay;
//    linearAccel.z = az;
//    angularAccel.x = gx;
//    angularAccel.y = gy;
//    angularAccel.z = gz;
    
    orientation.x = roll;
    orientation.y = pitch;
    orientation.z = heading;

//    linearAccel_topic.publish( &linearAccel );
//    angularAccel_topic.publish( &angularAccel );
    orientation_topic.publish( &orientation );
    
    nh.spinOnce();

    // increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
}

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
