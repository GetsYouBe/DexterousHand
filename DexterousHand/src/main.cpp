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

int radTostep(double);
int move_time(int,int,int,int);


SMS_STS SerialServo;
IMU_device IMU_arm(2,0x29);//IMUを扱うインスタンスを生成
Serial_debug debug(Serial);

double aim_qua_w=1;
double aim_qua_x=0;
double aim_qua_y=0;
double aim_qua_z=0;

int posX = 90;
int posY = 90;
int posZ = 90;
int servo_v = 3400;
int servo_a = 50;

void setup(){
  Serial.begin(115200);
  Serial1.begin(1000000,SERIAL_8N1, rxPin, txPin);
  SerialServo.pSerial = &Serial1;
 
  Serial.println("start");

  pinMode(13, OUTPUT);
  
  //bool hand_conection = IMU_hand.begin();//begin()でIMUとの通信開始
  bool arm_conection = IMU_arm.begin();

  if(!arm_conection){
    Serial.println("no BNO055(arm) detected");
    while(1){};
  }else{
    IMU_arm.setExtCrystalUse(true);
    Serial.println("IMU_arm setting complete");
  }

  SerialServo.RegWritePosEx(1,2047,3400,50);//初期位置に移動
  SerialServo.RegWritePosEx(2,2047,3400,50);
  SerialServo.RegWritePosEx(3,2047,3400,50);
  

  delay(500);
  
}

void loop(){
  posX = SerialServo.ReadPos(1);
  posZ = SerialServo.ReadPos(2);
  posY = SerialServo.ReadPos(3);

  imu::Quaternion qua_now = IMU_arm.getQuat();
  imu::Quaternion qua_aim(aim_qua_w,aim_qua_x,aim_qua_y,aim_qua_z);
  imu::Quaternion diff = diffQuaterniopn(qua_now,qua_aim);
  imu::Vector<3> vec = convert(diff);//x-z-y

  debug.WebSerialprint((float)diff.w(),(float)diff.x(),(float)diff.y(),(float)diff.z(),3,3,3,3);

  int dx = radTostep(vec[0]);//オイラー角をサーボのステップ数に変換
  int dz = radTostep(vec[1]);//これは動作量であって絶対角度ではない
  int dy = radTostep(vec[2]);

  SerialServo.RegWritePosEx(1,posX+dx,servo_v,servo_a);//サーボを駆動
  SerialServo.RegWritePosEx(2,posZ+dz,servo_v,servo_a);
  SerialServo.RegWritePosEx(3,posY+dy,servo_v,servo_a);

  delay(  max( move_time(posX+dx,posX,servo_v,servo_a), max(move_time(posZ+dz,posZ,servo_v,servo_a), move_time(posY+dy,posY,servo_v,servo_a) ) )  );//サーボを動作時間待機

  
}

int radTostep(double rad){
  return (int)(rad/2 * 3.14) * 4095;
}

int move_time(int Pos,int prePos, int v, int a){
    return ((Pos - prePos)/v) * 1000 + (v/(a*100)) * 1000;
}