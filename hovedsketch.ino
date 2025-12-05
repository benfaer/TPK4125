float angle = 0;
float motorSpeed = 20;
int targetSpeed = 0;
int motorDelta = 0;

void setup() {
  Serial.begin(115200);
  setupSensor();
  setupPID();
  setupMotor();
  setupBLE();

}

void loop() {
  sensorTask();
  pidTask();
  motorTask();
  bleTask();

}
