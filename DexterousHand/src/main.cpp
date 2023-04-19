#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <Arduino.h>
#include <SCServo.h>
#include "imu.h"
#include "print_debug.h"
#include "control.h"
#include <SoftwareSerial.h>

#define rxPin 16
#define txPin 17

int radTostep(double);
int move_time(int,int,int,int);
int StepRange_con(int);

SoftwareSerial DebugSerial(10, 11);

SMS_STS SerialServo;
Adafruit_BNO055 IMU_arm(55,0x28);//IMUを扱うインスタンスを生成
Serial_debug debug(DebugSerial);


double aim_qua_w=1;
double aim_qua_x=0;
double aim_qua_y=0;
double aim_qua_z=0;

int posX = 90;
int posY = 90;
int posZ = 90;
int servo_v = 3400;
int servo_a = 50;

bool LED = true;

void setup(){
  DebugSerial.begin(4800);
  Serial.begin(1000000);
  SerialServo.pSerial = &Serial;
 
  DebugSerial.write("start");

  pinMode(13, OUTPUT);
  
  //bool hand_conection = IMU_hand.begin();//begin()でIMUとの通信開始
  bool arm_conection = IMU_arm.begin();

  if(!arm_conection){
    DebugSerial.println("no BNO055(arm) detected");
    while(1){}
  }else{
    IMU_arm.setExtCrystalUse(true);
    DebugSerial.println("IMU_arm setting complete");
  }

  SerialServo.RegWritePosEx(1,2047,3400,50);//初期位置に移動
  SerialServo.RegWritePosEx(2,2047,3400,50);
  SerialServo.RegWritePosEx(3,2047,3400,50);
  SerialServo.RegWriteAction();

  delay(500);
  
}

void loop(){
  digitalWrite(13,LED);
  LED = !LED;
  posX = StepRange_con(SerialServo.ReadPos(1));//0 < (step) < 4054
  posZ = StepRange_con(SerialServo.ReadPos(2));
  posY = StepRange_con(SerialServo.ReadPos(3));

  imu::Quaternion qua_now = IMU_arm.getQuat();
  imu::Quaternion qua_aim(aim_qua_w,aim_qua_x,aim_qua_y,aim_qua_z);
  imu::Quaternion diff = diffQuaterniopn(qua_now,qua_aim);
  imu::Vector<3> vec = convert(diff);//x-z-y 0 < (rad) < 2pi

  debug.WebSerialprint((float)diff.w(),(float)diff.x(),(float)diff.y(),(float)diff.z(),3,3,3,3);

  vec[0] = RadRange_con(vec[0]);
  vec[1] = RadRange_con(vec[1]);
  vec[2] = RadRange_con(vec[2]);

  int dx = radTostep(vec[0]);//オイラー角をサーボのステップ数に変換
  int dz = radTostep(vec[1]);//これ差分になる絶対角度で、センサーが根元にしかついていないから先端が水平になっていても小さくならない。モーターの目標角度になる。
  int dy = radTostep(vec[2]);//0 < (step) < 4054

  SerialServo.RegWritePosEx(1,dx,servo_v,servo_a);//サーボを駆動
  SerialServo.RegWritePosEx(2,dz,servo_v,servo_a);//絶対角度指定
  SerialServo.RegWritePosEx(3,dy,servo_v,servo_a);
  SerialServo.RegWriteAction();

  delay(  max( move_time(dx,posX,servo_v,servo_a), max(move_time(dz,posZ,servo_v,servo_a), move_time(dy,posY,servo_v,servo_a) ) )  );//サーボを動作時間待機
  
}

int radTostep(double rad){
  return (int)(rad/(2 * 3.14)) * 4095;
}

int move_time(int Pos,int prePos, int v, int a){
    return ((Pos - prePos)/v) * 1000 + (v/(a*100)) * 1000;
}

int StepRange_con(int raw){//step
    return raw-2047;
}

double RadRange_con(double raw){
  if(raw <= 3.14) return raw;
  else return (raw - 3.14) - 3.14;
}