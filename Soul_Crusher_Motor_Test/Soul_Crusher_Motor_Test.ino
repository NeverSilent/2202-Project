#define servoLeft 25
#define servoRight 23

#define yellowLeftA 18
#define yellowLeftB 4
#define yellowRightA 21
#define yellowRightB 22

#define pot1 A4
#define pot2 A7

int WheelSpeed;
int ServoPos;

void setup() {
  // put your setup code here, to run once:
  ledcAttachPin(yellowLeftA, 1);
  ledcAttachPin(yellowLeftB, 2);
  ledcAttachPin(yellowRightA, 3);
  ledcAttachPin(yellowRightB, 4);

  ledcAttachPin(servoLeft, 5);
  ledcAttachPin(servoRight, 6);
  
  ledcSetup(1, 20000, 8);
  ledcSetup(2, 20000, 8);
  ledcSetup(3, 20000, 8);
  ledcSetup(4, 20000, 8);
  ledcSetup(5, 20000, 8);
  ledcSetup(6, 20000, 8);
}

void loop() {
  WheelSpeed = map(analogRead(pot1), 0, 4096, 130, 255);
  ServoPos = map(analogRead(pot2), 0, 4096, 0, 60);
  
  
  if(WheelSpeed >= 135){ //make a zone of full stop
    ledcWrite(1, WheelSpeed);
    ledcWrite(2, 0);
    ledcWrite(3, WheelSpeed);
    ledcWrite(4, 0);
    ledcWrite(5, DDP(ServoPos));
    ledcWrite(6, DDP(ServoPos));
    
  } else {
    ledcWrite(1, 0);
    ledcWrite(2, 0);
    ledcWrite(3, 0);
    ledcWrite(4, 0);    
  }
}

long DDP(int deg) { //degrees to PWM
  const long minDutyCycle = 1675;            // duty cycle for 0 degrees
  const long maxDutyCycle = 8050;            // duty cycle for 180 degrees

  long dutyCycle = map(deg, 0, 180, minDutyCycle, maxDutyCycle);  // convert to duty cycle

  return dutyCycle;
}
