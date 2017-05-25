/*
  chenlvzhou 2017.5.20
  http://clz.me/
*/
#include "Arduino.h"
#include "Servo_Motor.h"

#include <Servo.h>
#include <CurieIMU.h>
#include <MadgwickAHRS.h>

Servo_Motor::Servo_Motor(int pinL, int pinR)
{
  _pinL = pinL;
  _pinR = pinR;
  _directionL = FORWARD;
  _directionR = FORWARD;
  _speedL = 0;
  _speedR = 0;
  _stateL = STOP;
  _stateR = STOP;
  _calibrationTime = 0;
  // servoL.attach(_pinL);
  // servoR.attach(_pinR);
  CurieIMU.begin();
  CurieIMU.setGyroRate(100);
  CurieIMU.setGyroRange(250);
  CurieIMU.setAccelerometerRate(100);
  CurieIMU.setAccelerometerRange(2);
  filter.begin(100);
}

void Servo_Motor::calibrate() {
  Serial.println("Calibration begin...");
  float a[CALNUM];
  uint64_t b[CALNUM];
  uint64_t c = 0;
  a[0] = getHeading();
  b[0] = micros();
  for (int i = 1; i < CALNUM; i++) {
    do {
      a[i] = getHeading();
      b[i] = micros();
    } while (a[i] - a[i - 1] < 1);
    c = c + (b[i] - b[i - 1]);
  }
  _calibrationTime = c / (CALNUM-1);
  Serial.println("Calibration end");
  Serial.print("calibrationTime:");
  Serial.println(_calibrationTime);
}



void Servo_Motor::run(int speed = 100)
{
  motorL(speed);
  motorR(speed);
}

void Servo_Motor::backrun(int speed = 100)
{
  motorL(-speed);
  motorR(-speed);
}

void Servo_Motor::motorL(int speed)
{
  if (speed == 0) {
    stopL();
    _speedL = 0;
  }
  else {
    if (speed > 100) {
      speed = 100;
    }
    else if (speed < -100) {
      speed = -100;
    }
    _speedL = speed;
    int val = map(_speedL, -100, 100, -500, 500);
    if (_stateL == STOP) {
      servoL.attach(_pinL);
      _stateL = RUNNING;
    }
    if (_directionL == FORWARD) {
//      Serial.print("motorL val:");
//      Serial.println(val);
      servoL.writeMicroseconds(1500 + val);
    }
    else {
      servoL.writeMicroseconds(1500 - val);
    }
  }
}

void Servo_Motor::motorR(int speed)
{
  if (speed == 0) {
    stopR();
    _speedR = 0;
  }
  else {
    if (speed > 100) {
      speed = 100;
    }
    else if (speed < -100) {
      speed = -100;
    }
    _speedR = speed;
    int val = map(_speedR, -100, 100, -500, 500);
    if (_stateR == STOP) {
      servoR.attach(_pinR);
      _stateR = RUNNING;
    }
    if (_directionR == FORWARD) {
//      Serial.print("motorR val:");
//      Serial.println(val);
      servoR.writeMicroseconds(1500 - val);
    }
    else {
      servoR.writeMicroseconds(1500 + val);
    }
  }
}

void Servo_Motor::turnLeft(float angle, int speed)
{
  angle = angle - OFFSETANGLE1;
  float oldAngle = getHeading();
  uint64_t calibrateStartTime = micros();
  float newAngle = oldAngle;
  while (newAngle - oldAngle < angle) {
    motorL(-speed);
    motorR(speed);
    newAngle = getHeading();
    newAngle = newAngle - ((micros() - calibrateStartTime) / _calibrationTime);
#ifdef DEBUG
    Serial.println(newAngle);
#endif
    if ((newAngle < oldAngle) && (oldAngle - newAngle > OFFSETANGLE2)) newAngle = newAngle + 360;
  }
#ifdef DEBUG
  Serial.println("turn left ok");
#endif
  stop();
}

void Servo_Motor::turnLeft(float angle)
{
  turnLeft(angle, 100);
}

void Servo_Motor::turnRight(float angle, int speed)
{
  angle = angle - OFFSETANGLE1;
  float oldAngle = getHeading();
  uint64_t calibrateStartTime = micros();
  float newAngle = oldAngle;
  while (oldAngle - newAngle < angle) {
    motorL(speed);
    motorR(-speed);
    newAngle = getHeading();
    newAngle = newAngle - ((micros() - calibrateStartTime) / _calibrationTime);
#ifdef DEBUG
    Serial.println(newAngle);
#endif
    if ((newAngle > oldAngle) && (newAngle - oldAngle > OFFSETANGLE2)) newAngle = newAngle - 360;
  }
#ifdef DEBUG
  Serial.println("turn right ok");
#endif
  stop();
}

void Servo_Motor::turnRight(float angle)
{
  turnRight(angle, 100);
}

void Servo_Motor::turnLeft()
{
  motorL(-100);
  motorR(100);
}

void Servo_Motor::turnRight()
{
  motorL(100);
  motorR(-100);
}

void Servo_Motor::setDirectionL(bool direction)
{
  _directionL = direction;
}

void Servo_Motor::setDirectionR(bool direction)
{
  _directionR = direction;
}

void Servo_Motor::brake()
{
  //反转,速度较慢暂时可以不加
  stop();
}

void Servo_Motor::stopL()
{
  servoL.writeMicroseconds(1500);
  delay(50);
  servoL.detach();
  _stateL = STOP;
}

void Servo_Motor::stopR()
{
  servoR.writeMicroseconds(1500);
  delay(50);
  servoR.detach();
  _stateR = STOP;
}

void Servo_Motor::stop()
{
  stopL();
  stopR();
}

float Servo_Motor::getHeading() {
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  float heading;
  while (!CurieIMU.dataReady());
  CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
  ax = convertRawAcceleration(aix);
  ay = convertRawAcceleration(aiy);
  az = convertRawAcceleration(aiz);
  gx = convertRawGyro(gix);
  gy = convertRawGyro(giy);
  gz = convertRawGyro(giz);
  filter.updateIMU(gx, gy, gz, ax, ay, az);
  heading = filter.getYaw();
  return heading;
}

float Servo_Motor::convertRawAcceleration(int aRaw) {
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}

float Servo_Motor::convertRawGyro(int gRaw) {
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}


