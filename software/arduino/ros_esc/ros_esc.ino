/*
  Author: Sebastian Hardin    hardinse@oregonstate.edu
*/

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include "ESC.h"
#include "PCA9685.h"


#define LED_PIN 13
#define ESCA_PIN 9
#define ESCA_PWM_CHANNEL 0
#define ESCB_PWM_CHANNEL 1
#define ESCC_PWM_CHANNEL 2
#define ESCD_PWM_CHANNEL 3
#define PWM_FREQ 100


//the node handler
ros::NodeHandle nh;

//the esc
//ESC escA(ESCA_PIN, 1000, 2000, 500);  //pin number, min value, max value, arm value

//the PCA9685 module (mux)
PCA9685 pwmController;

//the values to be read in from converted_thrust, used on escs
int valA, valB, valC, valD;

//callback used when subscribing to converted_thrust
void callback(const std_msgs::Int16MultiArray& array)
{
  //map A (from -100 to 100) to a range of 1000 to 2000 for esc
  //val = map(array.data[0], -100, 100, 1000, 2000);

  //map to A B C D, in a range of [-100, 100] to a range of [0, 4096) for the pca9685
  valA = map(array.data[0], -100, 100, 0, 4095);
  valB = map(array.data[1], -100, 100, 0, 4095);
  valC = map(array.data[2], -100, 100, 0, 4095);
  valD = map(array.data[3], -100, 100, 0, 4095);
}

//subscribe to converted_thrust, use callback function called callback
ros::Subscriber<std_msgs::Int16MultiArray> sub("converted_thrust", &callback);

void setup()
{ 
  //ESC.h stuff
  //pinMode(LED_PIN, OUTPUT);     //set up LED, on pin 13
  //escA.arm();                   //arm the esc
  //digitalWrite(LED_PIN, HIGH);  //light up LED once esc is armed

  //PCA9685.h stuff
  Serial.begin(115200);                      //begin Serial interface, initialize bit/s rate
  Wire.begin();  //begin i2c Wire interface (expects the PCA9685 to be connected to A4-SDA and A5-SCL)
  pwmController.resetDevices();              //reset all devices on i2c line
  pwmController.init();                      //initialize pca9685 module
  pwmController.setPWMFrequency(PWM_FREQ);   //set pwm frequency of the PCA9685
  
  //prep the ROS side of things (subscribe to converted_thrust)
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{  
  //ESC.h stuff
  //escA.speed(val);   //assign val to the esc

  //PCA9685.h stuff
  pwmcontroller.setChannelPWM(ESCA_PWM_CHANNEL, valA);
  pwmcontroller.setChannelPWM(ESCB_PWM_CHANNEL, valB);
  pwmcontroller.setChannelPWM(ESCC_PWM_CHANNEL, valC);   //assign values to escs
  pwmcontroller.setChannelPWM(ESCD_PWM_CHANNEL, valD);

  //ROS stuff
  nh.spinOnce();

  
  delay(15);
}
