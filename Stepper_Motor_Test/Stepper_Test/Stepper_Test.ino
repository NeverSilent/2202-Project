/*
  Demonstrates set up and control of a stepper motor using the DRV8825 stepper motor driver

  Assumes use of MSEduino:
  R1 (connect jumper to JP2) controls step rate
  PB1 (connect jumper JP13) to controls direction
  Connect SDA (JP11) to direction pin of DRV8825
  Connect SCLK (JP11) to step pin of DRV8825

  Note that this code does not implement any acceleration or deceleration. The stepper may "lock up" if the
  direction is changed when the motor is spinning quickly. 

  Language: Arduino (ESP32)
  Author: Michael Naish
  Date: 21/02/08

  Rev 1 - Initial verison

*/

//#define OUTPUT_ON                           // uncomment to turn on output debugging information

const int potPin = 32;                        // select the analog pin used for the potentiometer (R1)
const int buttonPin = 27;                     // select digital pin for built-in pushbutton PB1 (JP13 required)
const int dirPin = 21;                        // select digital pin for step direction, uses GPIO21, I2C_DA (SDA pin)
const int stepPin = 22;                       // select digital pin for step pin, uses GPIO22, I2C_CLK (SLCK pin)

int val;                                      // input value from the analog pin
unsigned long prevMicrosec = 0;               // start time for delay cycle, in milliseconds
unsigned long curMicrosec = 0;                // current time, in milliseconds
unsigned long stepCount = 0;                  // number of steps
unsigned long stepRate;                       // step rate in microseconds

boolean btDir = true;                         // step direction

const long clDebounceDelay = 50;              // button debounce delay in milliseconds
unsigned long ulLastDebounceTime;             // start time for debounce, in milliseconds
int iButtonState;                             // button state
int iLastButtonState = HIGH;                  // 

void setup() {
#ifdef OUTPUT_ON
  Serial.begin(9600);
#endif
  
  pinMode(buttonPin, INPUT_PULLUP);            // Assign digital input for pushbutton and turn on internal pullup
  pinMode(dirPin, OUTPUT);                     // Assign output for direction
  pinMode(stepPin, OUTPUT);                    // Assign output for step 
}

void loop() {
   int iButtonValue = digitalRead(buttonPin);  // read value of push button 1
   if (iButtonValue != iLastButtonState) {     // if value has changed
      ulLastDebounceTime = millis();           // reset the debouncing timer
   }

  if ((millis() - ulLastDebounceTime) > clDebounceDelay) {
     if (iButtonValue != iButtonState) {       // if the button state has changed
     iButtonState = iButtonValue;              // update current button state

      // only toggle direction if the new button state is LOW
      if (iButtonState == LOW) {
        btDir = !btDir;
        digitalWrite(dirPin, btDir);           // set direction
      }
    }
  }
  iLastButtonState = iButtonValue;             // store button state    

  curMicrosec = micros();                      // get the current time in milliseconds
  if (curMicrosec - prevMicrosec > stepRate) { // check to see if elapsed time matched the desired delay
    prevMicrosec = curMicrosec;           
    stepCount++;                               // 
    val = analogRead(potPin);                  // reads the value of the potentiometer (value between 0 and 4096)
    stepRate = map(val, 0, 4096, 100, 100000); // scale it into delay between steps in microseconds
    digitalWrite(stepPin, stepCount & 1);      // toggle step pin (0 if stepCount is even, 1 if stepCount is odd)

#ifdef OUTPUT_ON
    Serial.print("Dir: ");
    Serial.print(btDir);
    Serial.print(", rate: ");
    Serial.print(stepRate);
    Serial.print(", count: ");
    Serial.println(stepCount);
#endif
  }  
}
