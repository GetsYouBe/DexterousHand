#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <Arduino.h>
#include <SCServo.h>
#include "imu.h"
#include "print_debug.h"
#include "control.h"
#include <SoftwareSerial.h>
#include <MsTimer2.h>
#include "emg_calib.h"

#define rxPin 16
#define txPin 17





SoftwareSerial DebugSerial(10, 11);
EMG_CALIB myemg(DebugSerial);

SMS_STS SerialServo;
Adafruit_BNO055 IMU_arm(55,0x28);//IMUを扱うインスタンスを生成
Serial_debug debug(DebugSerial);

double aim_qua_w=1;//目標クォータニオン
double aim_qua_x=0;
double aim_qua_y=0;
double aim_qua_z=0;
imu::Quaternion qua_aim(aim_qua_w,aim_qua_x,aim_qua_y,aim_qua_z);

int posX = 90;
int posY = 90;
int posZ = 90;
int servo_v = 3400;
int servo_a = 50;
int mode = 0;// 0:開＋OFF　１：閉＋OFF　２閉＋ON

bool LED = true;

int radTostep(double rad){
  float x = (rad/(2 * 3.14)) * 4095;
  return (int)x;
}

int move_time(int Pos,int prePos, int v, int a){
    return ((Pos - prePos)/v) * 1000 + (v/(a*100)) * 1000;
}

int StepRange_con(int raw){//step
    return raw-2047;
}

int StepRange_res(int raw){
  return raw + 2027;
}

double RadRange_con(double raw){
  if(raw <= 3.14) return raw;
  else return (raw - 3.14) - 3.14;
}


volatile int counter=0;
volatile float result_i=0;
volatile float result=0;
volatile float sum=0;
volatile const float loop_count=20.0;
float judge_0to1=2.72;
float judge_1to2=3.43;
float gain_i = 10; //n/10 (0 <= n <= 10)

void Interrupt(){
  if(counter>=loop_count){//測定値を積分か平均をとる
    result_i=sum/loop_count;
    //Serial.println(result,3);
    counter=0;
    sum=0;
    result = result_i * (gain_i/10) + (analogRead(EMG)/1023.0 * 5.0) * ((10-gain_i)/10);
  }else{
    sum+=analogRead(EMG)/1023.0 * 5.0;//5V換算
    counter++;
  }
}

  

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

  IMU_arm.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P2);
  IMU_arm.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P2);

  MsTimer2::set(10,Interrupt);
  MsTimer2::start();

  SerialServo.RegWritePosEx(1,2047,3400,50);//初期位置に移動
  SerialServo.RegWritePosEx(2,2047,3400,50);
  SerialServo.RegWritePosEx(3,2047,3400,50);
  SerialServo.RegWriteAction();

  delay(500);
  
}

void loop(){
  DebugSerial.print("mode:");
  DebugSerial.print(mode);
  DebugSerial.print("EMG:");
  DebugSerial.print(result);
  DebugSerial.println("");

  if(DebugSerial.available() > 0){
    myemg.threshold_calib(judge_0to1,judge_1to2);
    DebugSerial.readString();
    delay(1000);
  }

  switch (mode)
  {
  case 0/*義手を開いて水平制御OFF*/:{
    SerialServo.RegWritePosEx(4,2047,servo_v,servo_a);//指を閉じる
    SerialServo.RegWritePosEx(5,2047,servo_v,servo_a);

    SerialServo.RegWriteAction();
    move_time(2600,2047,servo_v,servo_a);

    if(judge_0to1 <= result){//閾値より高かったらmode変更
    mode = 1;
    }

  break;}

  case 1/*義手を閉じて水平制御OFF*/:{
    SerialServo.RegWritePosEx(4,2600,servo_v,servo_a);//指を閉じる
    SerialServo.RegWritePosEx(5,1400,servo_v,servo_a);

    SerialServo.RegWriteAction();
    move_time(2600,2047,servo_v,servo_a);

    qua_aim = IMU_arm.getQuat();

    if(result < judge_0to1){//もし閾値より筋電が低くなっていたらmodeを変える
    mode = 2;
    }else if(judge_1to2 <= result){//手を閉じて水平制御ON
    mode = 0;
    }

  break;} 

  case 2/*義手を閉じて水平制御をする*/:{
    digitalWrite(13,LED);
    LED = !LED;



    posX = StepRange_con(SerialServo.ReadPos(1));//-2027 < (step) < 2027
    posZ = StepRange_con(SerialServo.ReadPos(2));
    posY = StepRange_con(SerialServo.ReadPos(3));

    imu::Quaternion qua_now = IMU_arm.getQuat();
    
    imu::Quaternion diff = diffQuaterniopn(qua_aim,qua_now);
    imu::Vector<3> vec = convert(diff);//x-z-y 0 < (rad) < 2pi

    //debug.WebSerialprint((float)diff.w(),(float)diff.x(),(float)diff.y(),(float)diff.z(),3,3,3,3);

    vec.x() = RadRange_con(vec.x());// -pi < rad < pi
    vec.z() = RadRange_con(vec.z());
    vec.y() = RadRange_con(vec.y());

    int dx = radTostep(vec.x());//オイラー角をサーボのステップ数に変換
    int dz = radTostep(vec.z());//これ差分になる絶対角度で、センサーが根元にしかついていないから先端が水平になっていても小さくならない。モーターの目標角度になる。
    int dy = radTostep(vec.y());//-2047 < (step) < 2047

    

    dx = constrain(StepRange_res(dx),1000,3000);
    dz = constrain(StepRange_res(dz),1500,2500);
    dy = constrain(StepRange_res(dy),1500,2700);// 0 < step < 4054
    
  
    /*
    DebugSerial.print(dx);
    DebugSerial.print(",");
    DebugSerial.print(dz);
    DebugSerial.print(",");
    DebugSerial.println(dy);
    */
    


    MsTimer2::stop();
    SerialServo.RegWritePosEx(1,dx,servo_v,servo_a);//手首サーボを駆動
    SerialServo.RegWritePosEx(2,4054-dz,servo_v,servo_a);//絶対角度指定
    SerialServo.RegWritePosEx(3,4054-dy,servo_v,servo_a);

    SerialServo.RegWritePosEx(4,2600,servo_v,servo_a);//指を閉じる
    SerialServo.RegWritePosEx(5,1400,servo_v,servo_a);

    SerialServo.RegWriteAction();
    MsTimer2::start();


    delay(  max( move_time(dx,posX,servo_v,servo_a), max(move_time(dz,posZ,servo_v,servo_a), move_time(dy,posY,servo_v,servo_a) ) )  );//サーボを動作時間待機

    if(result < judge_1to2){//手を閉じて水平制御OFF
      mode = 1;
    }

    break;}
  
  default:
    break;
  }
}