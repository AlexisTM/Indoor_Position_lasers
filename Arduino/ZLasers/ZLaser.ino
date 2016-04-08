#include "LidarObject.h"
#include "LidarController.h"
#include "I2CFunctions.h"

// Only if multiple UART port board
//#define USE_USBCON
#include <ros.h>
#include <laserpack/Distance.h>
//#include <laserpack/init.h>

#include <Wire.h>
#define WIRE400K true
/*** Defines : CONFIGURATION ***/
// Defines laser ready data
#define Z1_LASER_PIN 10
#define Z2_LASER_PIN 8
// Defines power enable lines of laser
#define Z1_LASER_EN 11
#define Z2_LASER_EN 9
//Define address of lasers
//Thoses are written during initialisation
// default address : 0x62
#define Z1_LASER_AD 0x64
#define Z2_LASER_AD 0x66

#define NUMBER_OF_LASERS 2

#define READINESS true

// Maximum datarate
#define DATARATE 100
// Actual wait between communications 100Hz = 10ms
#define DELAY_SEND_MICROS 1000000/DATARATE

// Lidars
static LidarController Controller;
static LidarObject LZ1;
static LidarObject LZ2;

// rate controller
long now, last;


// ROS communication
ros::NodeHandle nh;
laserpack::Distance   distance_msg;
ros::Publisher distpub("/lasers/raw", &distance_msg);

void beginLidars() {
  // Initialisation of the lidars objects
  LZ1.begin(Z1_LASER_EN, Z1_LASER_PIN, Z1_LASER_AD, 2, 'z');
  LZ2.begin(Z2_LASER_EN, Z2_LASER_PIN, Z2_LASER_AD, 2, 'Z');
  // Initialisation of the controller
  Controller.begin(WIRE400K);
  delay(10);
  Controller.add(&LZ1, 0);
  Controller.add(&LZ2, 1);
}

void beginROSComm(){
  nh.initNode();
  nh.advertise(distpub);
  pinMode(13, OUTPUT);
}

void setup() {
  Serial.begin(57600);
  while (!Serial);
  beginLidars();
  beginROSComm();
  last = micros();
}

void laserPublish(){
  static int16_t dataOut[2] = {0,0};
  static uint8_t statusOut[2] = {0,0};
  distance_msg.lasers_length = 2;
  distance_msg.status_length = 2;
  distance_msg.lasers = Controller.distances;
  distance_msg.status = Controller.statuses;
 
  distpub.publish( &distance_msg );
}

void loop() {
  nh.spinOnce();
  Controller.spinOnce();
  now = micros();
  if(now - last > DELAY_SEND_MICROS){
    last = micros();
    laserPublish();
  } 
}
