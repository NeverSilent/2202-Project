#define servoLeft 25
#define servoRight 23

#define yellowLeft 21 //Sda
#define yellowRight 22 //Sclk

#define pot2 A4

int WheelSpeed;
int ServoPos;

void setup() {
  Serial.begin(9600);
  
  // put your setup code here, to run once:
  ledcAttachPin(yellowLeft, 1);
  ledcAttachPin(yellowRight, 2);

  ledcAttachPin(servoLeft, 5);
  ledcAttachPin(servoRight, 6);
  
  ledcSetup(1, 20000, 8);
  ledcSetup(2, 20000, 8);
  ledcSetup(5, 50, 16);
  ledcSetup(6, 50, 16);
}

void loop() {
  ServoPos = map(analogRead(pot2), 0, 4096, 11, 88);

  Serial.print(ServoPos);
  Serial.print("\n");
  ledcWrite(5, DDP(ServoPos));
  ledcWrite(6, DDP(90 - ServoPos));
  
  //if(WheelSpeed >= 135){ //make a zone of full stop
    ledcWrite(1, 255);
    ledcWrite(2, 255); //opposiate +/- to force oppisate rotation hardwire
    
  /*} else {
    ledcWrite(1, 0);
    ledcWrite(2, 0);   
  }*/
}

int DDP(int deg) { //degrees to PWM
  const long minDutyCycle = 1675;            // duty cycle for 0 degrees
  const long maxDutyCycle = 8050;            // duty cycle for 180 degrees

  long dutyCycle = map(deg, 0, 180, minDutyCycle, maxDutyCycle);  // convert to duty cycle

  return dutyCycle;
}
