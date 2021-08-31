/*MSE 2202 
 * 
 * Western Engineering base code
 * 2020 05 13 E J Porter
 * 2021 04 06 JD Herlehy
 * + servos for the arm
 * + yellow motors for lifting the bot
 * + cases for driving around obstacale
 * + cases for checking if the IR was found
 * /// changed the const int to a define statment == less memory in the bot
 * + Added the parts that would need to be calibrated per robot
*/


//Pin assignments
#define ciHeartbeatLED 2
#define ciPB1 27     
#define ciPB2 26      
#define ciPot1 A4    //GPIO 32  - when JP2 has jumper installed Analog pin AD4 is connected to Poteniometer R1
#define ciPot2 A7
#define ciLimitSwitch 26
#define ciIRDetector 16
#define ciMotorLeftA 4
#define ciMotorLeftB 18
#define ciMotorRightA 19
#define ciMotorRightB 12
#define ciEncoderLeftA 17
#define ciEncoderLeftB 5
#define ciEncoderRightA 14
#define ciEncoderRightB 13
#define ciSmartLED 25

#define servoLeft 25
#define servoRight 23

#define yellowLeft 21 //Sda
#define yellowRight 22 //Sclk

//for the encoders
volatile uint32_t vui32test1;
volatile uint32_t vui32test2;

#include "0_Core_Zero.h"

#include <esp_task_wdt.h>

#include <Adafruit_NeoPixel.h>
#include <Math.h>
#include "Motion.h";
#include "MyWEBserver.h"
#include "BreakPoint.h"
#include "WDT.h";

void loopWEBServerButtonresponce(void);

const int CR1_ciMainTimer = 1000;
const int CR1_ciHeartbeatInterval = 500;
int CR1_ciMotorRunTime = 600;               //not const anymore since being able to change the timing provves useful
const long CR1_clDebounceDelay = 50;
const long CR1_clReadTimeout = 220;

unsigned char CR1_ucMainTimerCaseCore1;
uint8_t CR1_ui8LimitSwitch;

uint8_t CR1_ui8IRDatum;
uint8_t CR1_ui8WheelSpeed;
uint8_t CR1_ui8Adjuster;
uint8_t CR1_ui8LeftWheelSpeed;
uint8_t CR1_ui8RightWheelSpeed;

uint32_t CR1_u32Now;
uint32_t CR1_u32Last;
uint32_t CR1_u32Temp;
uint32_t CR1_u32Avg;

unsigned long CR1_ulLastDebounceTime;
unsigned long CR1_ulLastByteTime;

unsigned long CR1_ulMainTimerPrevious;
unsigned long CR1_ulMainTimerNow;

unsigned long CR1_ulMotorTimerPrevious;
unsigned long CR1_ulMotorTimerNow;
unsigned char ucMotorStateIndex = 0;
unsigned char beaconHit = 0;
unsigned char correction = 0;
unsigned char reverseSet = 0;                       ///To Know if the IR Beacon was hit or not, so the sequencing only runs once

unsigned long CR1_ulHeartbeatTimerPrevious;
unsigned long CR1_ulHeartbeatTimerNow;

boolean btHeartbeat = true;
boolean btRun = false;
boolean btToggle = true;
int iButtonState;
int iLastButtonState = HIGH;

// Declare our SK6812 SMART LED object:
Adafruit_NeoPixel SmartLEDs(2, 25, NEO_GRB + NEO_KHZ400);
// Argument 1 = Number of LEDs (pixels) in use
// Argument 2 = ESP32 pin number 
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

void setup() {
   Serial.begin(115200); 
   Serial2.begin(2400, SERIAL_8N1, ciIRDetector);  // IRDetector on RX2 receiving 8-bit words at 2400 baud
   
   Core_ZEROInit();

   WDT_EnableFastWatchDogCore1();
   WDT_ResetCore1();
   WDT_vfFastWDTWarningCore1[0] = 0;
   WDT_vfFastWDTWarningCore1[1] = 0;
   WDT_vfFastWDTWarningCore1[2] = 0;
   WDT_vfFastWDTWarningCore1[3] = 0;
   WDT_ResetCore1();
   WDT_vfFastWDTWarningCore1[4] = 0;
   WDT_vfFastWDTWarningCore1[5] = 0;
   WDT_vfFastWDTWarningCore1[6] = 0;
   WDT_vfFastWDTWarningCore1[7] = 0;
   WDT_ResetCore1();
   WDT_vfFastWDTWarningCore1[8] = 0;
   WDT_vfFastWDTWarningCore1[9] = 0;
   WDT_ResetCore1(); 

   setupMotion();
   pinMode(ciHeartbeatLED, OUTPUT);
   pinMode(ciPB1, INPUT_PULLUP);
   pinMode(ciLimitSwitch, INPUT_PULLUP);


   // put your setup code here, to run once:
   ledcAttachPin(yellowLeft, 10);
   ledcAttachPin(yellowRight, 11);

   ledcAttachPin(servoLeft, 12);
   ledcAttachPin(servoRight, 13);
  
   ledcSetup(10, 20000, 8);
   ledcSetup(11, 20000, 8);
   ledcSetup(12, 50, 16);
   ledcSetup(13, 50, 16);

   SmartLEDs.begin();                          // Initialize Smart LEDs object (required)
   SmartLEDs.clear();                          // Set all pixel colours to off
   SmartLEDs.show();                           // Send the updated pixel colours to the hardware
}

void loop()
{
  //WSVR_BreakPoint(1);

  int iButtonValue = digitalRead(ciPB1);       // read value of push button 1
  if (iButtonValue != iLastButtonState) {      // if value has changed
     CR1_ulLastDebounceTime = millis();        // reset the debouncing timer
  }

 if ((millis() - CR1_ulLastDebounceTime) > CR1_clDebounceDelay) {
    if (iButtonValue != iButtonState) {        // if the button state has changed
    iButtonState = iButtonValue;               // update current button state

     // only toggle the run condition if the new button state is LOW
     if (iButtonState == LOW)
     {
       ENC_ClearLeftOdometer();
       ENC_ClearRightOdometer();
       btRun = !btRun;
        Serial.println(btRun);
       // if stopping, reset motor states and stop motors
       if(!btRun)
       {
          ucMotorStateIndex = 0; 
          ucMotorState = 0;
          move(0);
       }
      
     }
   }
 }
 iLastButtonState = iButtonValue;             // store button state

 if(!digitalRead(ciLimitSwitch))
 {
  btRun = 0; //if limit switch is pressed stop bot
  ucMotorStateIndex = 0;
  ucMotorState = 0;
  move(0);
 }
 
 if (Serial2.available() > 0) {               // check for incoming data
    CR1_ui8IRDatum = Serial2.read();          // read the incoming byte
// Serial.println(iIncomingByte, HEX);        // uncomment to output received character
    CR1_ulLastByteTime = millis();            // capture time last byte was received
 }
 else
 {
    // check to see if elapsed time exceeds allowable timeout
    if (millis() - CR1_ulLastByteTime > CR1_clReadTimeout) {
      CR1_ui8IRDatum = 0;                     // if so, clear incoming byte
    }
 }
 CR1_ulMainTimerNow = micros();
 if(CR1_ulMainTimerNow - CR1_ulMainTimerPrevious >= CR1_ciMainTimer)
 {
   WDT_ResetCore1(); 
   WDT_ucCaseIndexCore0 = CR0_ucMainTimerCaseCore0;
   
   CR1_ulMainTimerPrevious = CR1_ulMainTimerNow;
    
   switch(CR1_ucMainTimerCaseCore1)  //full switch run through is 0.25mS
   {
    //###############################################################################
    case 0: 
    {
      
      if(btRun)
      {
       CR1_ulMotorTimerNow = millis();
       if(CR1_ulMotorTimerNow - CR1_ulMotorTimerPrevious >= CR1_ciMotorRunTime)   
       {   
         CR1_ulMotorTimerPrevious = CR1_ulMotorTimerNow;
         switch(ucMotorStateIndex)
         {
          case 0: //time to get away
          {
            ucMotorStateIndex = 1;
            ucMotorState = 0;
            move(0);
            break;
          }

          case 1:       /// first straight away
          {
            CR1_ciMotorRunTime = 2000; //set the time allocated for each case to 2 sec
            ENC_SetDistance(175, 175); //go forward a bit
            ucMotorState = 1; //forward
            CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
            CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
            ucMotorStateIndex = 2;
            break;
          }
          
          case 2:       /// first left turn
          {
            ENC_SetDistance(29, -29); //go left a bit                           /// Change the angle per robot
            ucMotorState = 2; //left                                            
            CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
            CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
            ucMotorStateIndex = 3;
            break;
          }
          
          case 3:       /// second straight away
          {
            ENC_SetDistance(240, 240); //go forward a bit
            ucMotorState = 1; //forward
            CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
            CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
            ucMotorStateIndex = 5;
            break;
          }
          
          case 5:       /// slow increments up third straight away inorder to hit the IR Beacon
          {
           CR1_ciMotorRunTime = 1000; //set the time allocated for each case to 1 sec
            if(CR1_ui8IRDatum == 0x55){
              ENC_SetDistance(40, 40); //go forward a bit if the IR has been seen
              ucMotorState = 1;   //forward
              CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
              CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed; 
            } else if((CR1_ui8IRDatum != 0x55)&&(CR1_ui8IRDatum != 0x41)){ //if the green isn't seen (beacon is red)
              ENC_SetDistance(3, -3); //turn a small amount to the left if the IR isn't found
              ucMotorState = 2; //left
              CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
              CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
            } //turning left results in a full circle, but if bot vears to the left while going straight, switch to turn right a little
            break;
          }

          case 6: //back up a little to give space for drum 1 to close on the rope
          {
            ENC_SetDistance(5, 5); //reverse a bit
            ucMotorState = 4; //reverse
            CR1_ui8LeftWheelSpeed = CR1_ui8WheelSpeed;
            CR1_ui8RightWheelSpeed = CR1_ui8WheelSpeed;
            ucMotorStateIndex = 7;
            break;
          }
          
          case 7: //full brake the drive motors, close drum 1 via arm
          {
            CR1_ciMotorRunTime = 3000; //set the time allocated for each case to 3 sec
            
             //stop the drive motors
             ledcWrite(2,255);
             ledcWrite(1,255);
             ledcWrite(4,255);
             ledcWrite(3,255);

             //engage the arm //11 is the up position, 88 is the down I think
             ledcWrite(12, DDP(88));  
             ledcWrite(13, DDP(90 - 88)); 
             ucMotorStateIndex = 8;
             break;
          }
          
         case 8: //Spin up the drum, to climb the rope
          {
            CR1_ciMotorRunTime = 22000; //set the time allocated for each case to 22 sec
             //run the drum
             ledcWrite(10, 255);
             ledcWrite(11, 255);
             
             ucMotorStateIndex = 9;
             break;
            }
                  
            case 9: //stop the drum
            {
             CR1_ciMotorRunTime = 3000; //set the time allocated for each case to 3 sec
             
             //stop the drum
             ledcWrite(10, 0);
             ledcWrite(11, 0);

             ucMotorStateIndex = 10;
             break;
            }

            case 10: //lower the arm
            {

             //lower the arm
             ledcWrite(12, DDP(11));
             ledcWrite(13, DDP(90 - 11));

             
             break;
            }

        }
      }
      CR1_ucMainTimerCaseCore1 = 1;
      
      break;
    }
    //###############################################################################
    case 1: 
    {
      //read pot 1 for motor speeds 
      CR1_ui8WheelSpeed = map(analogRead(ciPot1), 0, 4096, 130, 255);

      CR1_ui8Adjuster = map(analogRead(ciPot2), 0, 4096, 0, 100); //how much the speed gets put to the left and the right
      
      CR1_ucMainTimerCaseCore1 = 2;
      break;
    }
    //###############################################################################
    case 2: 
    {
      //average the encoder tick times
      ENC_Averaging();

      CR1_ucMainTimerCaseCore1 = 3;
      break;
    }
    //###############################################################################
    case 3: 
    {
      //move bot X number of odometer ticks
      if(ENC_ISMotorRunning())
      {
        //RightAdjust(CR1_ui8RightWheelSpeed, CR1_ui8Adjuster)
        //Check the Motion Library for the added Functions(LeftAdjuct, RightAdjust)
        MoveTo(ucMotorState, LeftAdjust(CR1_ui8LeftWheelSpeed, CR1_ui8Adjuster) + 12, RightAdjust(CR1_ui8RightWheelSpeed, CR1_ui8Adjuster) - 42); //older, 12,42 //new 0,12
      } 
   
      CR1_ucMainTimerCaseCore1 = 4;
      break;
    }
    //###############################################################################
    case 4:   
    {
     
      CR1_ucMainTimerCaseCore1 = 5;
      break;
    }
    //###############################################################################
    case 5: 
    {
      
      CR1_ucMainTimerCaseCore1 = 6;
      break;
    }
    //###############################################################################
    case 6:
    {    
      CR1_ucMainTimerCaseCore1 = 7;
      break;
    }
    //###############################################################################
    case 7: 
    {
       if (CR1_ui8IRDatum == 0x55) {                // if proper character is seen
         SmartLEDs.setPixelColor(0,0,25,0);         // make LED1 green with 10% intensity
       }
       else if (CR1_ui8IRDatum == 0x41) {           // if "hit" character is seen
         SmartLEDs.setPixelColor(0,25,0,25);        // make LED1 purple with 10% intensity
         // only when IR was in
          if(reverseSet == 0){
          ucMotorStateIndex = 6; //start to climb
          reverseSet = 1;
         }
       }
       else {                                       // otherwise
         SmartLEDs.setPixelColor(0,255,255,255);         // make LED1 red with 10% intensity
         SmartLEDs.setPixelColor(1,255,255,255);
       }
       SmartLEDs.show();                            // send updated colour to LEDs
          
      CR1_ucMainTimerCaseCore1 = 8;
      break;
    }
    //###############################################################################
    case 8: 
    {
    
      CR1_ucMainTimerCaseCore1 = 9;
      break;
    }
    //###############################################################################
    case 9: 
    {
  
      CR1_ucMainTimerCaseCore1 = 0;
      break;
    }

  }
 }

 // Heartbeat LED
 CR1_ulHeartbeatTimerNow = millis();
 if(CR1_ulHeartbeatTimerNow - CR1_ulHeartbeatTimerPrevious >= CR1_ciHeartbeatInterval)
 {
    CR1_ulHeartbeatTimerPrevious = CR1_ulHeartbeatTimerNow;
    btHeartbeat = !btHeartbeat;
    digitalWrite(ciHeartbeatLED, btHeartbeat);
   // Serial.println((vui32test2 - vui32test1)* 3 );
 }
 }
}

int DDP(int deg) { //degrees to PWM
  const long minDutyCycle = 1675;            // duty cycle for 0 degrees
  const long maxDutyCycle = 8050;            // duty cycle for 180 degrees

  long dutyCycle = map(deg, 0, 180, minDutyCycle, maxDutyCycle);  // convert to duty cycle

  return dutyCycle;
}
