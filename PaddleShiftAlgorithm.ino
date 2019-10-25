#include <Servo.h> // Servo Motor Library

Servo servo; // Declare the servo
int analogPinE = A0; // The pin to use for reading the potentiometer
int analogPinD = A1;
int solenoidPin = A2; // The pin to write to the solenoid
int interruptPinR = A3; // The pin which will provide the interrupt
int interruptPinL = A4;

int paddleFlag;
int biasE;
int biasD;


void setup() {
  // Setup the analogue pins and servo  
  pinMode(analogPinE,INPUT);
  pinMode(analogPinD,INPUT);
  servo.attach(9); // Servo is attached to pin 9

  // Setup the solenoid pin 
  pinMode(solenoidPin, OUTPUT);

  // Setup the interrupt
  attachInterrupt(digitalPinToInterrupt(interruptPinR),paddleUp_ISR, CHANGE); // UpShift
  attachInterrupt(digitalPinToInterrupt(interruptPinL),paddleUp_ISR, CHANGE); // DownShift
  
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
  // Interrupt signal from paddel
  // Get both biases
  shiftControl(paddleFlag, biasE, biasD); // Control Gear
                        

  //mughees to include the code below this comment
}

void paddleUp_ISR() // Need to see if this can be implemented
{
  paddleFlag = 1;
}
 
void paddleDown_ISR()
{
  paddleFlag = 0;
}

void shiftControl(int shift, int biasE, int biasD)
{
  clutchControl(1, biasE, biasD);
  solenoidControl(1);
  delay(1000); // Need to add verification of gear selection 
  solenoidControl(0);
  clutchControl(0, biasE, biasD);
}

void clutchControl(int engage, int biasE, int biasD)
{
  if(engage == 1)
  {
    servoControl(biasE,engage);
  }
  else if(engage == 0); 
  {
    servoControl(biasD,engage);
  }
  
}

void servoControl(int bias, int rotationDirection)
{
  Serial.print("Bias Value:");
  Serial.println(bias); 
  int angle;
  if(rotationDirection == 1) // Engage Clutch
  {
    for(angle = 0; angle < 180; angle += bias)
    {
      servo.write(angle);
    }
  }
  else if(rotationDirection == 0) // Disengage Clutch
  {
    for(angle = 180; angle > 0; angle -= bias)
    {
      servo.write(angle);
    }
  }

}

void solenoidControl(int shift)
{
  if(shift == 0) //Netural
  {
    digitalWrite(solenoidPin, LOW);
  }
  else if(shift == 1) // Push
  {
    digitalWrite(solenoidPin, HIGH);
  }
  else if(shift == 2) // Pull
  {
    // Need to figure out how the solenoid will pull
  }
}
