#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <Ticker.h>
#include <ESP32Servo.h>
#include <Arduino.h>
#include "imu.h"
#include "print_debug.h"



IMU_device IMU_arm(2,0x29);//IMUを扱うインスタンスを生成


void setup(){
  Serial.begin(115200);
  //Serial1.begin(1000000);
  Serial.println("start");

  pinMode(13, OUTPUT);
  
  //bool hand_conection = IMU_hand.begin();//begin()はIMUとの通信開始
  bool arm_conection = IMU_arm.begin();
  /*IMUの設定--------------------------------------------------------*/
  /*if(!hand_conection){
    Serial.println("no BNO055(hand) detected");
  }else{
    IMU_hand.setExtCrystalUse(true);
    Serial.println("IMU_hand setting complete");
  }*/
  

  if(!arm_conection){
    Serial.println("no BNO055(arm) detected");
  }else{
    IMU_arm.setExtCrystalUse(true);
    Serial.println("IMU_arm setting complete");
  }

  /*if(!hand_conection || !arm_conection){//どちらかが見つからなかったら停止
    Serial.println("system stop");
    while(1);
  }*/

  
}

void loop(){

  
}