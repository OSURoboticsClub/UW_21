#include <ESC.h>

//button pins
#define FWD_PIN 2
#define BWD_PIN 3

// esc full forward
#define ESC_FWD 2000

// esc full backward
#define ESC_BWD 1000

// esc neutral
#define ESC_NEU 1500

// initialize
int esc_signal = ESC_NEU;
int fwd_state = 0;
int bwd_state = 0;
ESC myESC(9, ESC_BWD, ESC_FWD, 500);

void setup() {
 // set up button pins
 pinMode(FWD_PIN, INPUT);
 pinMode(BWD_PIN, INPUT);

 // start the esc
 myESC.arm();

 // start serial communication
 Serial.begin(9600);

 // give time for the ESC to do its thing
 delay(1000);
}

void loop() {
 // read the state of each button
 fwd_state = digitalRead(FWD_PIN);
 bwd_state = digitalRead(BWD_PIN);

  // if FWD is pressed
 if(fwd_state == HIGH){
  // set signal to full forward
  esc_signal = ESC_FWD;
  // if BWD is pressed
 } else if(bwd_state == HIGH){
  // set signal to full backward
  esc_signal = ESC_BWD;
 }
 else {
  // else signal is neutral
  esc_signal = ESC_NEU;
 }

 // print the signal
 Serial.println(esc_signal);
 
 // send the signal to the ESC
 myESC.speed(esc_signal);
 
 delay(100);

}
