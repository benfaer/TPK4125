#include <Arduino.h>

extern "C" {
  void ledcSetup(uint8_t, double, uint8_t);
  void ledcAttachPin(uint8_t, uint8_t);
}

const int ain1Pin = 4;
const int ain2Pin = 18; 
const int bin1Pin = 5; 
const int bin2Pin = 6;
const int pwmaPin = 19; 
const int pwmbPin = 7; 

const int STBY = 6;

const int pwmChannelA = 0;
const int pwmChannelB = 1;
const int pwmFreq = 1000;
const int pwmResolution = 8;

const float motorOffset = 20;

void setupMotor() {
    pinMode(ain1Pin, OUTPUT);
    pinMode(ain2Pin, OUTPUT);
    pinMode(bin1Pin, OUTPUT);
    pinMode(bin2Pin, OUTPUT);
}


void setMotorSpeed(float speed, int in1Pin, int in2Pin, int pwmPin) {

    int channel = (pwmPin == pwmaPin) ? pwmChannelA : pwmChannelB;

    if (speed < 0) {
      // CCW
      digitalWrite(in1Pin, LOW);
      digitalWrite(in2Pin, HIGH);
      speed = -speed;
    } else {
      // CW
      digitalWrite(in1Pin, HIGH);
      digitalWrite(in2Pin, LOW);
    }
    float calibratedSpeed = min(motorOffset * speed, motorOffset + speed * (255 - motorOffset) / 255);
    calibratedSpeed = constrain(calibratedSpeed, -255, 255);

    analogWrite(pwmPin, calibratedSpeed);

    /* Serial.print("PWM value: ");
    Serial.println(calibratedSpeed); */

    analogWrite(pwmPin, calibratedSpeed);
}

void motorTask() {
  setMotorSpeed(motorSpeed + motorDelta, ain1Pin, ain2Pin, pwmaPin);
  setMotorSpeed(motorSpeed - motorDelta, bin1Pin, bin2Pin, pwmbPin);
  Serial.print("Motor A: ");
  Serial.print(motorSpeed + motorDelta);
  Serial.print(" | Motor B: ");
  Serial.println(motorSpeed - motorDelta);
}
