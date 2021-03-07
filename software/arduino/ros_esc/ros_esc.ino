#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include "ESC.h"

#define LED_PIN 13
#define ESCA_PIN 9

//the node handler
ros::NodeHandle nh;

//the esc
ESC escA(ESCA_PIN, 1000, 2000, 500);  //pin number, min value, max value, arm value

//the value to be read in from converted_thrust, used on esc
int val;

//callback used when subscribing to converted_thrust
void callback(const std_msgs::Int16MultiArray& array)
{
  //map A (from -100 to 100) to a range of 1000 to 2000 for esc
  val = map(array.data[0], -100, 100, 1000, 2000);
}

//subscribe to converted_thrust, use messageCb function
ros::Subscriber<std_msgs::Int16MultiArray> sub("converted_thrust", &callback);

void setup()
{ 
  pinMode(LED_PIN, OUTPUT);     //set up LED, on pin 13
  escA.arm();                   //arm the esc
  digitalWrite(LED_PIN, HIGH);  //light up LED once esc is armed
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  escA.speed(val);   //assign val to the esc
  nh.spinOnce();
  delay(15);
}
