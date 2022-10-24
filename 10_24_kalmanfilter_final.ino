/*
    Kalman Filter Example for MPU6050. Output for processing.
    Read more: http://www.jarzebski.pl/arduino/rozwiazania-i-algorytmy/odczyty-pitch-roll-oraz-filtr-kalmana.html
    GIT: https://github.com/jarzebski/Arduino-KalmanFilter
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/


#include <Wire.h>
#include <MPU6050.h>
#include <KalmanFilter.h>
#include "SevSeg.h"
#define green A1
#define blue A2
#define red A0

MPU6050 mpu;
SevSeg sevseg;

int target_value = 50;

KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);

float accPitch = 0;
float accRoll = 0;

float kalPitch = 0;
float kalRoll = 0;

void setup()
{
  Serial.begin(115200);

  // Initialize MPU6050
  while (!mpu.begin(MPU6050_SCALE_500DPS, MPU6050_RANGE_8G))
  {
    delay(500);
  }
  
  byte numDigits = 4;                                                  //Set up 7 Segment Display
  byte digitPins[] = {10, 11, 12, 13};
  byte segmentPins[] = {2, 3, 4, 5, 6, 7, 8, 9};
  sevseg.begin(COMMON_CATHODE, numDigits, digitPins, segmentPins); 
  sevseg.setBrightness(90);
  
  pinMode(green,OUTPUT);
  pinMode(blue,OUTPUT);
  pinMode(red,OUTPUT);
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();
}

void loop()
{
  Vector acc = mpu.readNormalizeAccel();
  Vector gyr = mpu.readNormalizeGyro();

  // Calculate Pitch & Roll from accelerometer (deg)
  accPitch = -(atan2(acc.XAxis, sqrt(acc.YAxis * acc.YAxis + acc.ZAxis * acc.ZAxis)) * 180.0) / M_PI;
  accRoll  = (atan2(acc.YAxis, acc.ZAxis) * 180.0) / M_PI;

  // Kalman filter
  kalPitch = kalmanY.update(accPitch, gyr.YAxis);
  kalRoll = kalmanX.update(accRoll, gyr.XAxis);

  if (abs(kalPitch) < target_value){
    analogWrite(blue,256);
    analogWrite(green,0);
  }else{
    analogWrite(green,256);
    analogWrite(blue,0);
  }

  if (!(220 > abs(kalRoll) && abs(kalRoll) > 140)){
    analogWrite(red,256);
  }else{
    analogWrite(red,0);
  }
  Serial.print(kalRoll);
  Serial.print("\n");
  sevseg.setNumber(kalPitch, 0);
  sevseg.refreshDisplay();
  
}
