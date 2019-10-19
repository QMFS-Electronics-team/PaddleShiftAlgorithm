


void setup() {
  // put your setup code here, to run once:
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


  //mughees to include the code below this comment
  
  
}

//function Bhargav function below

void servoControl(){
  
} 
