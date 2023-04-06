#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <Ticker.h>
#include <ESP32Servo.h>
#include <Arduino.h>
#include <SCServo.h>
#include "imu.h"
#include "print_debug.h"
#include "control.h"

#define rxPin 16
#define txPin 17
SMS_STS SerialServo;

IMU_device IMU_arm(2,0x29);//IMUを扱うインスタンスを生成

 
double aim_qua_w=1;
double aim_qua_x=0;
double aim_qua_y=0;
double aim_qua_z=0;

void setup(){
  Serial.begin(115200);
  Serial1.begin(1000000,SERIAL_8N1, rxPin, txPin);
  SerialServo.pSerial = &Serial1;
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
  delay(500)
  
}

void loop(){
  imu::Quaternion qua_now = IMU_arm.getQuat();
  imu::Quaternion qua_aim(aim_qua_w,aim_qua_x,aim_qua_y,aim_qua_z);
  imu::Quaternion diff = diffQuaterniopn(qua_now,qua_aim);
  imu::Vector<3> vec = convert(diff);//x-z-y
  
  vec[0] = radTostep(vec[0]);
  vec[1] = radTostep(vec[1]);
  vec[2] = radTostep(vec[2]);

  SerialServo.RegWritePosEx(1,vec[0],3400,50);
  SerialServo.RegWritePosEx(2,vec[1],3400,50);
  SerialServo.RegWritePosEx(3,vec[2],3400,50);

  delay(max())

  
}

int radTostep(double rad){
  return (int)(rad/2 * 3.14) * 4095;
}

int move_time(int step, int speed, int accel){
    
}