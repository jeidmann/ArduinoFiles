//DEFINE VARIABLES
const byte interruptPin = D6;           // digital pin for ball detection switch
const byte solenoidPin = D5;           // digital pin for solenoid
volatile unsigned long throws = 0;     
unsigned long oldthrows = 0;
unsigned long startMillis;
volatile unsigned long throwtime, throwlast, throwinterval, currentMillis; // timing variables

//SETUP MOTOR PINS
#define enA D1
#define in1 D2
#define in2 D3
#define enB D8
#define in3 D4
#define in4 D7
int SPEED=500; //speed of motors; note, for ESP8266, use a scale of 0-1023

void setup()
{
  Serial.begin(115200);
  pinMode(interruptPin, INPUT_PULLUP);  // set pin mode with pullup 
  attachInterrupt(digitalPinToInterrupt(interruptPin), &ThrowPulse, FALLING); // attach interrupt 
  pinMode(solenoidPin, OUTPUT);
  startMillis=millis();  
  digitalWrite(solenoidPin, LOW);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
} // end setup


// MAIN LOOP
void loop()  
{  
        if (throws != oldthrows) {
          digitalWrite(solenoidPin,HIGH); //turn solinoid ON
          currentMillis=millis();
          analogWrite(enA,SPEED); //designate speed motors will go
          analogWrite(enB,SPEED);
          digitalWrite(in1, HIGH); //designate direction of motors
          digitalWrite(in2, LOW);
          digitalWrite(in4, HIGH);
          digitalWrite(in3, LOW);
          delay(3000);
            if (currentMillis-startMillis>=1000) {
              Serial.print("\n");
              Serial.print("No. of throws:");
              Serial.println(throws);
              Serial.print("Time Elapsed (sec):");
              Serial.println((currentMillis-startMillis)/1000); //Track secs elapsed since last throw
              digitalWrite(solenoidPin, HIGH);
              delay(50);
              digitalWrite(solenoidPin, LOW); //Turn solenoid off
              delay(3000);
              oldthrows=throws;
              startMillis = currentMillis;
            }
          digitalWrite(in1,LOW);  //Turn of motors
          digitalWrite(in2,LOW);
          digitalWrite(in3,LOW);
          digitalWrite(in4,LOW);
        } 
    
}  // end main loop

void ThrowPulse() 
{
  noInterrupts();                     // disable Interrupts
  throwtime = millis();                // get current time
  throwinterval = throwtime - throwlast; // calc time since last pulse
  if (throwinterval > 200)             // ignore switch-bounce glitches less than 200 ms 
   {
     throws++;                        // increment counter, sum # of tips
     throwlast = throwtime;              // set up for next event
   }         
   interrupts();                       // Re-enable Interrupts
}
