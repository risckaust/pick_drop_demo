/*
 * gripper_joystick
 *
 * This sketch is for RISC lab demo.
 * It is for controlling the servos
 * in the gripper (hexacopter)
 * using joystick.
 *
 * Author: Asmaa Al-Saggaf
 * Date: 7/31/2018
 */

#include <Servo.h>
#include <ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
 
Servo servo1; Servo servo2; // create servo objects to control the servo motors

// set pin numbers:
int pushbutton = 2;            // the number of the pushbutton pin

// variables will change:
int buttonState = 0;           // variable for reading the pushbutton status
int pos = 0;                   // variable to store the servo position

// instantiate the node handle
ros::NodeHandle nh;

// ROS msg for button status
std_msgs::Bool button_msg;
ros::Publisher button_pub("gripper_status", &button_msg);

// callback
void MsgCb( const sensor_msgs::Joy& msg){
  if (msg.axes[5] < 0){
    nh.loginfo("Button 9 was clicked");
    nh.loginfo("Motors are moving to drop");
    servo1.writeMicroseconds(1700); // drop
    servo2.writeMicroseconds(1700); // drop
    delay(2000);
    servo1.writeMicroseconds(1100); // pick 
    servo2.writeMicroseconds(1100); // pick
    nh.loginfo("Ready to pick-up");
   
  }/*if cond leave*/
}/*function*/

// instantiate the subscribers
ros::Subscriber<sensor_msgs::Joy> sub("joy", &MsgCb );

void setup(){ 
  servo1.attach(9);          // attaches the servo on pin 9 to the servo object
  servo2.attach(12);        // attaches the servo on pin 10 to the servo object
  pinMode(pushbutton,INPUT_PULLUP);  // initialize the pushbutton pin as an input
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(button_pub);

  servo1.writeMicroseconds(1100); // pick
  servo2.writeMicroseconds(1100); // pick
  nh.loginfo("Ready to pick-up");
} 
 
void loop(){
  buttonState = digitalRead(pushbutton);  // read the state of the pushbutton value
  if (buttonState == LOW){                      // check if the pushbutton is not pressed.
    button_msg.data = true;
    nh.loginfo("Drop to pick-up again");
  }
  else{
    button_msg.data = false;
  }

  button_pub.publish(&button_msg);
  delay(10);
  nh.spinOnce();
  
  
} 
