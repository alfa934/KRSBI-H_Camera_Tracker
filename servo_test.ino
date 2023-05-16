#include <Servo.h>
#include "ros.h"
#include "std_msgs/Int32MultiArray.h"

Servo myServo;

// SETUP ROS
ros::NodeHandle nh;

int middle = 90;
int rightMax = 180;
int leftMax = 0;

int posX = middle;
int dx = 4;
void messageCb(const std_msgs::Int32MultiArray& pos_msg)
{
  // X = pos_msg.data[0]
  // Y = pos_msg.data[1]
  
  if(pos_msg.data[0] > 540)
  {
    if(posX <= rightMax)
      posX += dx;
  }
  else if(pos_msg.data[0] < 100)
  {
    if(posX >= leftMax)
      posX -= dx;
  }
  myServo.write(posX);
  delay(10);
}

ros::Subscriber<std_msgs::Int32MultiArray> sub( "servo_pos", &messageCb );

void setup() {
  // put your setup code here, to run once:
  myServo.attach(9);
  nh.initNode();
  nh.subscribe(sub);
  myServo.write(middle);
}

void loop()  {
  nh.spinOnce();
  delay(1);
}