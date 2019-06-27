#include "ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Bool.h"

int driveVals[8];
int clawVals[2];
int* arrayIter;
ros::NodeHandle nh;

void driveCallback(const std_msgs::Int32MultiArray& motorMSGHead)
{
  int i;
  arrayIter = motorMSGHead.data;
  for (i = 0; i < 8; i++)
  {
    driveVals[i] = *arrayIter;
    arrayIter++;
  }
}

void clawCallback(const std_msgs::Int32MultiArray& motorMSGHead)
{
  int i;
  arrayIter = motorMSGHead.data;
  for (i = 0; i < 2; i++)
  {
    clawVals[i] = *arrayIter;
    arrayIter++;
  }
}

void lightCallback(const std_msgs::Bool& desiredLightState)
{
    bool lightState = *desiredLightState.data;
}

void eStopCallback(const std_msgs::Bool& desiredEStopState)
{
    bool eStopState = *desiredEStopState.data;
}

ros::Subscriber<std_msgs::Int32MultiArray> driveSub("drive_values", driveCallback);
ros::Subscriber<std_msgs::Int32MultiArray> clawSub("claw_values", clawCallback);
ros::Subscriber<std_msgs::Bool> lightSub("light", lightCallback);
ros::Subscriber<std_msgs::Bool> eStopSub("e_stop", eStopCallback);

void setup() {
  nh.initNode();
  nh.subscribe(driveSub);
  nh.subscribe(clawSub);
  nh.subscribe(lightSub);
  nh.subscribe(eStopSub);
  Serial.begin(57600);
}

void loop() {
  nh.spinOnce();
  delay(1);
}



