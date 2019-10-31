#include <Servo.h> // Servo Motor Library

//_____________Variable Declarations_____________
Servo servo; // Servo Motor Object

// Potentiometer Pin
int analogPinE = A0;

// Solenoid Pins
int solenoidPinU = A2; // Downshift Solenoid Pin
int solenoidPinD = A3; // Upshift Solenoid Pin

// Interrupt Pins
int interruptPinR = 2; //A4; // The pin which will provide the interrupt
int interruptPinL = 3; //A5;

// Variables Declared
int paddleFlagDown;
int paddleFlagUp;
int bias;

void setup() 
{
  // Setup the analogue pins and servo  
  pinMode(analogPinE,INPUT);

  // Servo is attached to pin 9
  servo.attach(9);
  servo.write(0);

  // Setup the solenoid pins 
  pinMode(solenoidPinU, OUTPUT);
  pinMode(solenoidPinD, OUTPUT);

  // Setup the interrupts

  pinMode(interruptPinR, INPUT_PULLUP);
  pinMode(interruptPinL, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPinR),paddleUp_ISR, CHANGE); // UpShift
  attachInterrupt(digitalPinToInterrupt(interruptPinL),paddleDown_ISR, CHANGE); // DownShift

  // Set inital flag states
  paddleFlagUp = 0;
  paddleFlagDown = 0;

  // This will be used to print out debug information 
  Serial.begin(1200); 
}


//________________Main Loop________________

void loop() 
{
  // Interrupt signal from paddel
  paddleFlagUp = digitalRead(interruptPinR);
  paddleFlagDown = digitalRead(interruptPinL);
  
  // Get bias
  bias = 5;//map(analogRead(A0),0,1023,0,180);

  Serial.print("Paddle Flag Up: "+ (String)paddleFlagUp + "\n");
  Serial.print("Paddle Flag Down: "+ (String)paddleFlagDown + "\n");
  Serial.print("Bias: " + (String)bias +" Degrees/Cycle \n");
 

  //mughees to include the code below this comment
  if(paddleFlagUp == 1) // Up Shift
  { 
    shiftControl(1,bias);
    paddleFlagUp = 0;
  }
  
  
  if(paddleFlagDown == 1) // Down Shift
  { 
    shiftControl(2,bias);
    paddleFlagDown = 0;
  }
}


//____________Interrupt Service Routines____________


void paddleUp_ISR() // Need to see if this can be implemented
{
  //Serial.print("Up Shift Button Pressed\n");
  paddleFlagUp = 1;
}
 
void paddleDown_ISR()
{
  //Serial.print("Down Shift Button Pressed\n");
  paddleFlagDown = 1;
}


//________________Control Functions________________


/*int shift is for solenoid control 
 * 0,1,2 = Netural, Push, Pull
int bias is for clutch speed */
void shiftControl(int shift, int bias) //Controls The Gear Shifts
{
  clutchControl(1, bias); // Engage Clutch
  solenoidControl(shift); // Control Solenoid
  delay(1000);            // Delay to allow for the gear to change <-- Need to add verification of gear selection
  solenoidControl(0);     // Power off solenoid - Netural Position
  clutchControl(0,bias);  //Release Clutch
} // End shiftControl

/* int engage 0,1 = Disengage, Engage 
 * int bias is for clutch speed */
void clutchControl(int engage, int bias) // Controls The Clutch
{
  if(engage == 1)                 // Engage Clutch
  {
    servoControl(bias,engage);   // Move Servo Arm Accordingly 
  }
  else if(engage == 0);           // Disengage Clutch
  {
    servoControl(bias,engage);   // Move Servo Arm Accordingly
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

/*solenoidControl()
 * @brief: The solenoid has 3 modes push,pull and neutral. Writing a 1 to GPIO pins will drive a circuit to control the relays. Writing to pin: solenoidPinU cause an upshift and writing to pin: solenoidPinD causes downshift.
 * @params:
 * int shift control the actuation of the solenoid
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
    Serial.print("Pushing Solenoid\n");
    digitalWrite(solenoidPinU, HIGH);
    delay(10); // Delay added for testing
  }
  else if(shift == 2) // Pull
  {
    Serial.print("PullSolenoid\n");
    digitalWrite(solenoidPinD, HIGH);
    delay(10); // Delay added for testing
  }
} // End solenoidControl
