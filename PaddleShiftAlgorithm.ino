#include <Servo.h> // Servo Motor Library

Servo servo; // Declare the servo
int analogPin = A0; // The pin to use for reading the potentiometer


void setup() {
  // put your setup code here, to run once:

  // Setup the analogue pin and servo  
  pinMode(analogPin,INPUT); // Config analogue pin for input
  servo.attach(9); // Servo is attached to pin 9

  
  Serial.begin(9600); // this will be used to print out debug information 
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       
  servoControl();

  //mughees to include the code below this comment
  
  
}

void servoControl(){
  int poten = analogRead(A0); 
  Serial.print("Potentiometer Value:");
  Serial.println(poten); 
  servo.write(map(poten,0,1023,0,2018)); // Map 0-1023 with 0-180 (rotation)
    
} 
