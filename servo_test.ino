#include <Servo.h>
#include "ros.h"
#include "std_msgs/Int32MultiArray.h"

Servo myServo;

// SETUP ROS
ros::NodeHandle nh;
std_msgs::Int32MultiArray msg;

int middle = 90;
int rightMax = 180;
int leftMax = 0;

void messageCb(const std_msgs::Int32MultiArray& pos_msg)
{
  // X = pos_msg.data[0]
  // Y = pos_msg.data[1]
  
  if(pos_msg.data[0] > 540)
    myServo.write(rightMax);
  else if(pos_msg.data[0] < 100)
    myServo.write(leftMax);
}

ros::Subscriber<std_msgs::Int32MultiArray> sub( "servo_pos", &messageCb );

void setup() {
  // put your setup code here, to run once:
  myServo.attach(9);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()  {
  nh.spinOnce();
  delay(1);
}
