#include <QuickPID.h>

extern float angle;
extern float motorSpeed;
extern int motorDelta;
float targetAngle = 0;

float pidOutput = 0;

float Ku = 6.5;
float Tu = 1.38;

float Kp = 0.6 * Ku;
float Ki = (1.2 * Ku) / Tu;
float Kd = Ku * Tu * 0.075;

QuickPID anglePid(&angle, &pidOutput, &targetAngle);

unsigned long lastUnsaturatedTime = 0;
unsigned long lastBigAngleTime = 0;
bool pidEnabled = false;

void setupPID() {
    lastUnsaturatedTime = millis();
    lastBigAngleTime = millis();

    anglePid.SetMode(true);
    anglePid.SetOutputLimits(-255, 255);
    anglePid.SetSampleTimeUs(10000);
    anglePid.SetTunings(Kp, Ki, Kd); 
}

void pidTask() {
  // Sjekk hvor lenge det er siden vi har hatt en vinkel større enn 15 grader
  if (abs(angle) > 15) {
      lastBigAngleTime = millis();
  }

  Serial.print("Angle: ");
  Serial.print(angle);
  Serial.print(" | PID enabled: ");
  Serial.print(pidEnabled);
  Serial.print(" | Time since lastBigAngleTime: ");
  Serial.println(millis() - lastBigAngleTime);
  // Start opp pid-løkken om roboten har vært vertikal i mer enn 2 sekunder
  // og pid-løkken ikke har startet ennå
  if (!pidEnabled && millis() - lastBigAngleTime > 2000) {
      pidEnabled = true;
      anglePid.Reset();
      Serial.println("Starting up PID");
      Serial.print("Angle: ");
      Serial.println(angle);
  }

  // Sjekk hvor lengge det er siden vi har hatt et pådrag mindre enn 255
  if (abs(motorSpeed) < 255) {
      lastUnsaturatedTime = millis();
  }

  // Om det er mer enn 0.5s siden forrige gang vi hadde et pådrag mindre enn 255, stopp pid-løkken

  if (millis() - lastUnsaturatedTime > 500) {
      pidEnabled = false;
  }

  // Om vinkelen er større enn 45 grader, stopp pid-løkken umiddelbart
  if (abs(angle) > 45) {
      pidEnabled = false;
  }

  if (pidEnabled) {
      anglePid.Compute();
      motorSpeed = (int)pidOutput;
  } else {
      motorSpeed = 0;
      motorDelta = 0;
  }
}
