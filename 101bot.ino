#include "Servo_Motor.h"
unsigned long microsPerReading, microsPrevious;
Servo_Motor sm(13,12);

void setup(){
//  Serial.begin(115200);
//  while(!Serial);
  sm.calibrate();
  delay(3000);
  sm.run(100);
  delay(3000);
  sm.run(-100);
  delay(3000);
  sm.stop();

  delay(3000);
  sm.turnLeft(90);
  delay(3000);
  sm.stop();
  sm.turnRight(90);

  sm.stop();
  
  
//Serial.begin(115200);
//  while(!Serial);
//  delay(2000);
//  sm.calibrate();
//  Serial.println("3");
//  delay(1000);
//  Serial.println("2");
//  delay(1000);
//  Serial.println("1");
//  delay(1000);
//  Serial.println("gogog90");
//  sm.turnLeft(90);
//  Serial.println("3");
//  delay(1000);
//  Serial.println("2");
//  delay(1000);
//  Serial.println("1");
//  delay(1000);
//  Serial.println("gogog90");
//  sm.turnRight(90);
  
}

void loop(){
//    unsigned long microsNow;
//  microsNow = micros();
//  if (microsNow - microsPrevious >= microsPerReading) {
//    float newAngle = sm.getHeading();
//    Serial.println(newAngle);
    
//microsPrevious = microsPrevious + microsPerReading;
//  }
  
//   sm.run(100);
//   delay(1000);
  }
