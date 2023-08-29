#include <Servo.h>
#include "ros.h"
#include "std_msgs/Int32MultiArray.h"

Servo servoX, servoY;

// SETUP ROS
ros::NodeHandle nh;

// SERVO VARIABLES
int middle = 90;
int rightMax = 180;
int leftMax = 0;
int upMax = 45;
int downMax = 135;
int posX = middle;
int posY = middle;
int dx = 4;
int dy = 4;

void messageCb(const std_msgs::Int32MultiArray& pos_msg)
{
  // elements of the arrays (only has 2)
  // X = pos_msg.data[0]
  // Y = pos_msg.data[1]
  
  int X = pos_msg.data[0];
  int Y = pos_msg.data[1];
  
  /* This is for the "X" axis servo */
  if(X < 100)
  {
    if(posX <= rightMax)
      posX += dx;
  }
  else if(X > 540)
  {
    if(posX >= leftMax)
      posX -= dx;
  }
  // This is for the Y axis servo
  if(Y < 100) 
  {
    if(posY <= downMax)
      posY += dy;
  }
  else if(Y > 380)
  {
    if(posY >= upMax)
      posY -= dy;
  }
    
  servoX.write(posX);
  servoY.write(posY);

  delay(10);
}

ros::Subscriber<std_msgs::Int32MultiArray> sub( "servo_pos", &messageCb );

void setup() {
  // put your setup code here, to run once:
  servoX.attach(9);
  servoY.attach(10);
  nh.initNode();
  nh.subscribe(sub);
  servoX.write(middle);
  servoY.write(middle);
}

void loop()  {
  nh.spinOnce();
  delay(1);
}
