#include "ros.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/UInt16MultiArray.h"



unsigned int motorVals[8];
unsigned int* arrayIter;

ros::NodeHandle nh;




void arrayCallback(const std_msgs::UInt16MultiArray& motorMSGHead)
{
  int i;
  arrayIter = motorMSGHead.data;
  for (i = 0; i < 8; i++)
  {
    motorVals[i] = *arrayIter;
    arrayIter++;
  }
  Serial.println("Values Received\n");

}

ros::Subscriber<std_msgs::UInt16MultiArray> sub("array", arrayCallback);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  Serial.begin(57600);
}

void loop() {
  nh.spinOnce();
  delay(1);
}



