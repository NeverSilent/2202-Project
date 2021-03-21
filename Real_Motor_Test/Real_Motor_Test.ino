#define ciMotorLeftA 4
#define ciMotorLeftB 18
#define ciMotorRightA 19
#define ciMotorRightB 12
#define pot A4

int WheelSpeed;

void setup() {
  // put your setup code here, to run once:
  ledcAttachPin(ciMotorLeftA, 1);
  ledcAttachPin(ciMotorLeftB, 2);
  ledcAttachPin(ciMotorRightA, 3);
  ledcAttachPin(ciMotorRightB, 4);
  
  ledcSetup(1, 20000, 8);
  ledcSetup(2, 20000, 8);
  ledcSetup(3, 20000, 8);
  ledcSetup(4, 20000, 8);
}

void loop() {
  WheelSpeed = map(analogRead(pot), 0, 4096, 130, 255);
  
  if(WheelSpeed >= 135){
    ledcWrite(1, WheelSpeed);
    ledcWrite(2, 0);
    ledcWrite(3, WheelSpeed);
    ledcWrite(4, 0);
  } else {
    ledcWrite(1, 0);
    ledcWrite(2, 0);
    ledcWrite(3, 0);
    ledcWrite(4, 0);    
  }
}
