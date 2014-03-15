#include <Servo.h>

/**
  * ESC values to be configured
  */
#define MAX_ESC 180
#define MIN_ESC 90
#define STOP_ESC 90

/**  
  * Arduino Pin configuration
  */
#define ESC 9
#define BLUE_LED 13
#define RED_LED 12
#define INPUT_BUTTON 7

/**
  * Serial USB Baud Rate
  */
#define SERIAL_BAUDRATE 9600

/**
  * Program variables
  */
Servo esc;
int velocity;


void setup(){
  Serial.begin(SERIAL_BAUDRATE);
  
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(RED_LED, LOW);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(INPUT_BUTTON, INPUT);
  digitalWrite(INPUT_BUTTON, HIGH);  //Set pull-up on INPUT_BUTTON
  
  esc.attach(ESC);
  delay(100);
  
  // Set max value
  esc.write(MAX_ESC);  
  digitalWrite(RED_LED, HIGH);
  while(digitalRead(INPUT_BUTTON) == HIGH) ;
  
  // Set the min value
  esc.write(MIN_ESC);
  digitalWrite(BLUE_LED, HIGH);
  delay(300);
  while(digitalRead(INPUT_BUTTON) == LOW) ;
  
  esc.write(STOP_ESC);
  digitalWrite(RED_LED, LOW);
  delay(7000);
  
  velocity = STOP_ESC;
  digitalWrite(BLUE_LED, LOW);
}


void loop(){
  Serial.println(velocity);
  esc.write(velocity);
  velocity = (velocity + 5)%(MAX_ESC - MIN_ESC) + MIN_ESC;
  delay(500);
}


