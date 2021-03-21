const int ciMotorLeftA = 4;
const int ciMotorLeftB = 18;
const int ciMotorRightA = 19;
const int ciMotorRightB = 12;


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
  ledcWrite(1, 250);
  ledcWrite(2, 0);
  ledcWrite(3, 250);
  ledcWrite(4, 0);
}
