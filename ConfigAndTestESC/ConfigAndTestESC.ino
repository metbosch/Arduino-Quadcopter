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
#define ESC 3
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
  Serial.println("Do not connect ESC power");
  
#ifdef DEBUG_WITH_LEDS
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(RED_LED, LOW);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(INPUT_BUTTON, INPUT);
  digitalWrite(INPUT_BUTTON, HIGH);  //Set pull-up on INPUT_BUTTON
#endif
  
  esc.attach(ESC);
  delay(100);
  Serial.println("ESC attached and will be setted to MAX value");
  
  // Set max value
  esc.write(MAX_ESC);
  Serial.println("ESC setted to MAX value, waiting for incomming signal"); 
  
#ifdef DEBUG_WITH_LEDS
  digitalWrite(RED_LED, HIGH);
  while(digitalRead(INPUT_BUTTON) == HIGH) ;
#else
  while(!Serial.available()) ;
  Serial.read();
  Serial.flush();
#endif

  Serial.println("Signal received");
  
  // Set the min value
  esc.write(MIN_ESC);
  Serial.println("ESC setted to MIN value, waiting for incomming signal");
  
#ifdef DEBUG_WITH_LEDS
  digitalWrite(BLUE_LED, HIGH);
  delay(300);
  while(digitalRead(INPUT_BUTTON) == LOW) ;
#else
  while(!Serial.available()) ;
  Serial.read();
  Serial.flush();
#endif
  
  Serial.println("ESC will be setted to STOP value");
  esc.write(STOP_ESC);
#ifdef DEBUG_WITH_MOTORS
  digitalWrite(RED_LED, LOW);
#endif
  delay(7000);
  
  velocity = STOP_ESC;
#ifdef DEBUG_WITH_MOTORS
  digitalWrite(BLUE_LED, LOW);
#endif

  Serial.println("TO SET ESC STOP VALUE SEND SIGNAL WITH SERIAL PORT");
}


void loop(){
  Serial.println(velocity);
  esc.write(velocity);
  velocity = (velocity + 5)%(MAX_ESC - MIN_ESC) + MIN_ESC;
  delay(1000);
  if (Serial.available()) {
    esc.write(STOP_ESC);
    while (1) ;
  }
}


