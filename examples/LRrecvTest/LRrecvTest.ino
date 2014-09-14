/*****
 *
 *   LRrecvTest - Version 0.1.
 *
 *   LRrecvTest.ino Copyright 2014 by D. L. Ehnebuske 
 *   License terms: Creative Commons Attribution-ShareAlike 3.0 United States (CC BY-SA 3.0 US) 
 *                  See http://creativecommons.org/licenses/by-sa/3.0/us/ for specifics. 
 *
 *   Sketch to demonstrate the use of the LRremote library by using it to display the received keypresses from the Sparkfun
 *   little red IR remote control (https://www.sparkfun.com/products/11759).
 *
 *****/

#include <LRremote.h>

/****
 *
 * Constants for the button codes transmitted by the SparkFun Little Red Remote Control
 *
 ****/
#define SFIRR_POWER  0x10EFD827                // "Power" button IR code
#define SFIRR_A      0x10EFF807                // "A" button IR code
#define SFIRR_B      0x10EF7887                // "B" button IR code
#define SFIRR_C      0x10EF58A7                // "C" button IR code
#define SFIRR_UP     0x10EFA05F                // "Up" button IR code
#define SFIRR_DOWN   0x10EF00FF                // "Down" button IR code
#define SFIRR_LEFT   0x10EF10EF                // "Left" button IR code
#define SFIRR_RIGHT  0x10EF807F                // "Right" button IR code
#define SFIRR_SELECT 0x10EF20DF                // "Select" button IR code

#define RECV_PIN (3)                           // Arduino pin to which the IR receiver is attached
LRremote remote(RECV_PIN);                     // Instantiate an LRremote object to represent the IR remote/receiver pair

/****
 *
 * Set up the parameters for the onButton function. There are three of them: 
 *   1) An array of the IR remote control button codes we are interested in receiving
 *   2) A parallel array of button function pointers
 *   3) An integer that tells how many entries there are in the two arrays
 *
 ****/
long code[]          = {SFIRR_POWER, SFIRR_A, SFIRR_B, SFIRR_C, SFIRR_UP, SFIRR_DOWN, SFIRR_LEFT, SFIRR_RIGHT, SFIRR_SELECT};
void (*fPointer[])() = {fPower,      fA,      fB,      fC,      fUp,      fDown,      fLeft,      fRight,      fSelect};
int codeCount = sizeof(code)/sizeof(code[0]);

/****
 *
 * Invoked once each time the power comes up or the Arduino is reset
 *
 ****/

void setup()
{
  Serial.begin(9600);                                             // Start the serial monitor port
  remote.enable();                                                // Enable timer interrupts
  Serial.println("LRrecvTest Version 0.10: Ready to recieve.");   // Say we're ready to go
}

/****
 *
 *   Button functions -- one for each button on the remote. Invoked (by remote.onButton()) when the user presses the
 *   corresponding button on the remote.
 *
 ****/

//  Power button
void fPower() {
  Serial.println("Power button.");
}

//  "A" button
void fA() {
  Serial.println("'A' button.");
}

//  "B" button
void fB() {
  Serial.println("'B' button.");
}

//  "C" button
void fC() {
  Serial.println("'C' button.");
}

// "Up" button
void fUp() {
  Serial.println("'Up' button.");
}

// "Down" button
void fDown() {
  Serial.println("'Down' button.");
}

// "Left" button
void fLeft() {
  Serial.println("'Left' button.");
}

// "Right" button
void fRight() {
  Serial.println("'Right' button.");
}

//  Select
void fSelect () {
  Serial.println("'Select' button.");
}

/****
 *
 * Invoked over and over as fast as possible. Read and print the IR remote button presses.
 *
 ****/

void loop() {

  remote.onButton(code, fPointer, codeCount);          // Invoke fPointer[i] when code[i] has been received
}
