#include <Servo.h> // Servo Motor Library

//_____________Variable Declarations_____________
Servo servo; // Servo Motor Object

// Potentiometer Pins
int analogPinE = A0;
int analogPinD = A1;

// Solenoid Pins
int solenoidPinU = A2; // Downshift Solenoid Pin
int solenoidPinD = A3; // Upshift Solenoid Pin

// Interrupt Pins
int interruptPinR = A4; // The pin which will provide the interrupt
int interruptPinL = A5;


// Variables Declared
int paddleFlag;
int biasE;
int biasD;


void setup() 
{
  // Setup the analogue pins and servo  
  pinMode(analogPinE,INPUT);
  pinMode(analogPinD,INPUT);
  
  servo.attach(9); // Servo is attached to pin 9

  // Setup the solenoid pins 
  pinMode(solenoidPinU, OUTPUT);
  pinMode(solenoidPinD, OUTPUT);

  // Setup the interrupts
  attachInterrupt(digitalPinToInterrupt(interruptPinR),paddleUp_ISR, CHANGE); // UpShift
  attachInterrupt(digitalPinToInterrupt(interruptPinL),paddleUp_ISR, CHANGE); // DownShift
  
  Serial.begin(9600); // this will be used to print out debug information 
}


//________________Main Loop________________

void loop() {
  // Interrupt signal from paddel
  // Get both biases
  shiftControl(paddleFlag, biasE, biasD); // Control Gear
  

  //mughees to include the code below this comment
  if(paddleFlag) {
    clutchControl(1, 5, 5);
    solenoidControl(1);
    delay(100);
    solenoidControl(0);
    clutchControl(0, 5, 5);
  }
  
  
  if(paddleFlag) {
    clutchControl(1, 5, 5);
    solenoidControl(2);
    delay(100);
    solenoidControl(0);
    clutchControl(0, 5, 5);
  }
}


//____________Interrupt Service Routines____________


void paddleUp_ISR() // Need to see if this can be implemented
{
  paddleFlag = 1;
}
 
void paddleDown_ISR()
{
  paddleFlag = 0;
}


//________________Control Functions________________


/*int shift is for solenoid control 
 * 0,1,2 = Netural, Push, Pull
int bias is for clutch speed */
void shiftControl(int shift, int biasE, int biasD) //Controls The Gear Shifts
{
  clutchControl(1, biasE, biasD); // Engage Clutch
  solenoidControl(shift);         // Push Solenoid For Up Shift
  delay(1000);                    // Delay to allow for the gear to change <-- Need to add verification of gear selection 
  solenoidControl(0);             // Power off solenoid - Netural Position
  clutchControl(0, biasE, biasD); //Release Clutch
} // End shiftControl

/* int engage 0,1 = Disengage, Engage 
 * int bias is for clutch speed */
void clutchControl(int engage, int biasE, int biasD) // Controls The Clutch
{
  if(engage == 1)                 // Engage Clutch
  {
    servoControl(biasE,engage);   // Move Servo Arm Accordingly 
  }
  else if(engage == 0);           // Disengage Clutch
  {
    servoControl(biasD,engage);   // Move Servo Arm Accordingly
  }
  
} // End clutchControl

/* int bias is for the speed of the servo 
 * int rotationDirection is the direction the arm moves 
 * 0,1 = 0-180, 180-0 */
void servoControl(int bias, int rotationDirection) // Rotates Servo Arm 
{
  int angle;
  if(rotationDirection == 1)                    // Engage Clutch
  {
    for(angle = 0; angle < 180; angle += bias)  // Rotate Servo To Engage Clutch
    {
      servo.write(angle);
    }
  }
  else if(rotationDirection == 0)               // Disengage Clutch
  {
    for(angle = 180; angle > 0; angle -= bias)  // Rotate Servo To Disengage Clutch
    {
      servo.write(angle);
    }
  }

} // End servoControl

/*int shift control the actuation of the solenoid
 * 0,1,2, = Netural, Push, Pull */
void solenoidControl(int shift) // Activate Solenoid To Change Gear
{
  if(shift == 0) //Netural
  {
    digitalWrite(solenoidPinU, LOW);
    digitalWrite(solenoidPinD, LOW);
  }
  else if(shift == 1) // Push
  {
    digitalWrite(solenoidPinU, HIGH);
  }
  else if(shift == 2) // Pull
  {
    digitalWrite(solenoidPinD, HIGH);
  }
} // End solenoidControl
