#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <Ticker.h>
#include <ESP32Servo.h>
#include <Arduino.h>
#include <SCServo.h>
#include "imu.h"
#include "print_debug.h"

#define rxPin 16
#define txPin 17


IMU_device IMU_arm(2,0x29);//IMUを扱うインスタンスを生成


void setup(){
  Serial.begin(115200);
  Serial1.begin(1000000,SERIAL_8N1, rxPin, txPin);
  Serial.println("start");

  pinMode(13, OUTPUT);
  
  //bool hand_conection = IMU_hand.begin();//begin()はIMUとの通信開始
  bool arm_conection = IMU_arm.begin();

  if(!arm_conection){
    Serial.println("no BNO055(arm) detected");
    while(1){};
  }else{
    IMU_arm.setExtCrystalUse(true);
    Serial.println("IMU_arm setting complete");
  }

  
}

void loop(){
  
  
}