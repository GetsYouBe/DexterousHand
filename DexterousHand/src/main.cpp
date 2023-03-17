#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <Ticker.h>
#include <ESP32Servo.h>
#include <Arduino.h>
#include "imu.h"
#include "print_debug.h"


//MU_device IMU_hand(1,0x28);
IMU_device IMU_arm(2,0x29);
Serial_scan Serial_receive;
webserial web_debug(IMU_arm);

bool flag = true;

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

  web_debug.WebSerialprint();

  Serial_receive.receive();//シリアル通信から読み込み

  uint8_t sys,gyro,accel,mag;

  IMU_arm.getCalibration(&sys,&gyro,&accel,&mag);//キャリブレーションの状態を取得
  S_print("sys",sys);
  S_print("gyro",gyro);
  S_print("accel",accel);
  S_print("mag",mag,END);

  if(Serial_receive.get() == 's'){
    if(sys == 3 && gyro == 3 && accel == 3 &&  mag == 3 && flag){
      IMU_arm.Save_calib();
      S_print("calibration is saved",END);
    }else{
      S_print("calibration not completed",END);
    }
  }

  if(Serial_receive.get() == 'e'){
      uint8_t buff[22];
      IMU_arm.get_calib(buff);
      S_print("sensor_offset",buff,22);
  }
}

