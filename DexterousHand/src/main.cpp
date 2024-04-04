#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>//IMUのライブラリ

#include <Arduino.h>
#include <SCServo.h>
#include <SoftwareSerial.h>//シリアルサーボとarduinoのライブラリ

#include "imu.h"
#include "print_debug.h"
#include "calculate.h"
#include "emg_calib.h"
#include "utility.h"
#include "CountTimer.h"//オリジナルライブラリ

#define rxPin 16
#define txPin 17//ソフトウェアシリアルで使う予定だったピン

#define GripStrength 150//握力閾値
#define GripPosLimit 300//指の位置閾値
#define GripDefaultPos 0//指の初期位置

#define jump_case1 true//筋電での動作切り替えで、case1(指閉じて水平制御OFF)を飛ばすcase0-case2だけにする


HardwareSerial SerialForServo(PC_11,PC_10);//シリアルサーボ用のシリアル通信
EMG_CALIB myemg(Serial2);

SMS_STS SerialServo;
Adafruit_BNO055 IMU_arm(55,0x29);//IMUを扱うインスタンスを生成
Serial_debug debug(Serial2);
CountTimer Ctimer;

double aim_qua_w=1;//目標クォータニオンの各要素
double aim_qua_x=0;
double aim_qua_y=0;
double aim_qua_z=0;
imu::Quaternion qua_aim(aim_qua_w,aim_qua_x,aim_qua_y,aim_qua_z);//目標クォータニオン生成

int posX = 90;//モーター初期位置
int posY = 90;
int posZ = 90;
int STS3032_v = 3600;//モーター速度
int STS3032_a = 80;//モーター加速度
int STS3215_v = 3600;
int STS3215_a = 200;
 

int mode = 0;// 0:開＋OFF　１：閉＋OFF　２閉＋ON
int Gripload_L;//ID 5 +　モーターにかかる負荷の大きさ 向きに注意
int Gripload_R;//ID 4 -
int Grippos_R = 0;//指の角度
int Grippos_L = 0;
int GripSpeed = 10;
unsigned long wait_time = 0;//ループ周期調整用

bool LED = true;



volatile int counter=0;//　筋電関連
volatile float result_i=0;
volatile float result=0;
volatile float sum=0;
volatile const float loop_count=1.0;
float judge_0to1=1.8;//筋電閾値
float judge_1to2=2.3;
float judge_0to2=3.1;
float n12 = 5.0;//閾値設定のパラメータ
float n23 = 5.0;
float n13 = 7.0;
float gain_i = 10; //n/10 (0 <= n <= 10)
EulerOrder axisOrder = EulerOrder::XYZ;//関節の座標系指定

void Interrupt();//割り込んで筋電位取得(使ってない)

void CloseHand();//手を閉じる

void OpenHand();

void setup(){
  Serial2.begin(115200);
  SerialForServo.begin(1000000);
  SerialServo.pSerial = &SerialForServo;
  
  while(!Serial2) delay(100);//シリアル通信の確立まで待機（Arduino純正ボード以外では必須）
  
  Serial2.write("start");

  pinMode(13, OUTPUT);
  
  //bool hand_conection = IMU_hand.begin();//begin()でIMUとの通信開始
  bool arm_conection = IMU_arm.begin();

  if(!arm_conection){
    Serial2.println("no BNO055(arm) detected");
    while(1){}
  }else{
    IMU_arm.setExtCrystalUse(true);
    Serial2.println("IMU_arm setting complete");
  }

  
  
  uint8_t mag_calib;
  uint8_t gyro_calib;
  uint8_t sys_calib;
  uint8_t accel_calib;
  
  do//IMUのコンパスの初期化（これを待たずに制御始めると途中で角度が大きく変動して暴走）
  {
    Serial2.print("calibrating.........");
    Serial2.print(mag_calib);
    Serial2.println("of3");
    IMU_arm.getCalibration(&sys_calib,&gyro_calib,&accel_calib,&mag_calib);
  } while (mag_calib != 3);

  while (!SerialForServo) delay(1);

  SerialServo.RegWritePosEx(1,2047,3400,50);//初期位置に移動
  SerialServo.RegWritePosEx(2,2047,3400,50);
  SerialServo.RegWritePosEx(3,2047,3400,50);
  SerialServo.RegWritePosEx(4,2047,3400,50);
  SerialServo.RegWritePosEx(5,2047,3400,50);
  SerialServo.RegWriteAction();

  delay(500);

  IMU_arm.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P3);//IMUの軸、座標設定
  IMU_arm.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P3);

  debug.StopDebug();
}

void loop(){
  while (!Serial2 || !SerialForServo){delay(10);digitalWrite(13,HIGH);}//通信確立まで待機
  digitalWrite(13,LOW);

  result = analogRead(EMG)/1023.0 * 3.3;
  
  Serial2.print("EMG:");
  Serial2.print(result);
  Serial2.print(",");
  Serial2.print("Min:");
  Serial2.print(0.0);
  Serial2.print(",");
  Serial2.print("Max:");
  Serial2.print(3.3);
  /*
  Serial2.print("mode:");//debug
  Serial2.print(mode);
  Serial2.print(" ");

  Serial2.print("Gripload_L:");
  Serial2.print(Gripload_L);
  Serial2.print(" ");
  
  Serial2.print("Gripload_R:");
  Serial2.print(Gripload_R);
  Serial2.print(" ");
  */

  String SoftSerial_str = debug.read_Serial();

  if(SoftSerial_str == "calib"){//シリアル通信で""calib""を受信したら、閾値のキャリブレーションを起動する
    myemg.threshold_calib(judge_0to1, judge_1to2, judge_0to2, n12, n23, n13);
    Serial2.readString();//受信バッファをクリア
    delay(1000);
  }

  if(SoftSerial_str == "emg"){
    Serial2.print("judge_0to1 : ");
    Serial2.print(judge_0to1);
    Serial2.print("judge_1to2 : ");
    Serial2.print(judge_1to2);
    Serial2.print("judge_0to2 : ");
    Serial2.print(judge_0to2);
    Serial2.readString();
    delay(1000);
  }

  

  if(SoftSerial_str == "1"){
    result = judge_0to1 + 0.1;
  }else if(SoftSerial_str == "2"){
    result = judge_0to2 + 0.1;
  }else if(SoftSerial_str == "0"){
    result = 0;
  }else if(SoftSerial_str == "n+"){
    if(n13  <= 9) n13++;
  }else if(SoftSerial_str == "n-"){
    if(n13 >= 1) n13--;
  }

  switch (mode)
  {
  case 0/*義手を開いて水平制御OFF*/:{
    OpenHand();
    qua_aim = IMU_arm.getQuat();
    if(jump_case1){
      if(judge_0to2 <= result){mode = 2;}
    }else{
      if(judge_0to1 <= result){mode = 1;}//閾値より高かったらmode変更
    }
    

  break;}

  case 1/*義手を閉じて水平制御OFF*/:{
    CloseHand();
    
    qua_aim = IMU_arm.getQuat();//目標クォータニオンを取得

    if(result < judge_0to1){//もし閾値より筋電が低くなっていたらmodeを変える
    mode = 0;
    }else if(judge_1to2 <= result){//手を閉じて水平制御ON
    mode = 2;
    }

  break;} 

  case 2/*義手を閉じて水平制御をする*/:{
    digitalWrite(13,LED);
    LED = !LED;

    CloseHand();

    if(Ctimer.wait(&wait_time)){

      posX = StepRange_con(SerialServo.ReadPos(1));//-2027 < (step) < 2027
      posZ = StepRange_con(SerialServo.ReadPos(2));
      posY = StepRange_con(SerialServo.ReadPos(3));

      debug.TimerDebug("GetPos");

      imu::Quaternion qua_now = IMU_arm.getQuat();//現在のクォータニオンを取得
      
      debug.TimerDebug("GetQua");

      imu::Quaternion diff = diffQuaterniopn(qua_aim,qua_now);

      debug.TimerDebug("GetDiff");

      imu::Vector<3> vec = convertEuler(diff,axisOrder);//x-z-y 0 < (rad) < 2pi

      debug.TimerDebug("Convert");
      //debug.WebSerialprint((float)diff.w(),(float)diff.x(),(float)diff.y(),(float)diff.z(),3,3,3,3);

      vec.x() = RadRange_con(vec.x());// -pi < rad < pi
      vec.z() = RadRange_con(vec.z());
      vec.y() = RadRange_con(vec.y());

      int dx = RadToStep(vec.x());//オイラー角をサーボのステップ数に変換
      int dz = RadToStep(vec.z());//これ差分になる絶対角度で、センサーが根元にしかついていないから先端が水平になっていても小さくならない。モーターの目標角度になる。
      int dy = RadToStep(vec.y());//-2047 < (step) < 2047


      //dx = constrain(StepRange_res(dx),1000,3000);
      //dz = constrain(StepRange_res(dz),1500,2500);
      //dy = constrain(StepRange_res(dy),1500,2700);// 0 < step < 4054
      
      /*
      Serial2.print("dx:");
      Serial2.print(dx);
      Serial2.print(" ");
      Serial2.print("dz:");
      Serial2.print(dz);
      Serial2.print(" ");
      Serial2.print("dy:");
      Serial2.print(dy);
      Serial2.print(" ");
      */
      debug.TimerDebug("getMovement");

      switch (axisOrder)
      {
      case EulerOrder::XYZ:
        dx = constrain(StepRange_res(dx),1000,3000);
        dy = constrain(StepRange_res(dy),1500,2500);
        dz = constrain(StepRange_res(dz),1500,2700);// 0 < step < 4054  
        SerialServo.RegWritePosEx(1,dx-1000,STS3215_v,STS3215_a);//手首サーボを駆動
        SerialServo.RegWritePosEx(2,dy,STS3215_v,STS3215_a);//絶対角度指定
        SerialServo.RegWritePosEx(3,4054-dz,STS3032_v,STS3032_a);  
        break;
      
      case EulerOrder::XZY:
        dx = constrain(StepRange_res(dx),1000,3000);
        dz = constrain(StepRange_res(dz),1500,2500);
        dy = constrain(StepRange_res(dy),1500,2700);// 0 < step < 4054
        SerialServo.RegWritePosEx(1,dx,STS3215_v,STS3215_a);//手首サーボを駆動
        SerialServo.RegWritePosEx(2,4054-dz,STS3215_v,STS3215_a);//絶対角度指定
        SerialServo.RegWritePosEx(3,4054-dy,STS3032_v,STS3032_a);  
        break;

      default:
        break;
      }
    }
    /*
    Serial2.print("wait_time:");
    Serial2.print(wait_time);
    Serial2.print(" ");
    */
    wait_time = 5;
    Ctimer.wait(&wait_time);

    if(jump_case1){
      if(result < judge_0to2){mode = 0;}
    }else{
      if(result < judge_1to2){mode = 1;}//手を閉じて水平制御OFF
    }
    

    break;}
  
  default:
    break;
  }

  SerialServo.RegWriteAction();

  debug.TimerDebug("Send");
      //wait_time = max( move_time(dx,posX,STS3032_v,STS3032_a), max(move_time(dz,posZ,STS3032_v,STS3032_a), move_time(dy,posY,STS3032_v,STS3032_a) ) );//サーボを動作時間待機
  
  Serial2.println("");
}

void Interrupt(){
  if(counter>=loop_count){//測定値を積分か平均をとる
    result_i=sum/loop_count;
    //Serial2.println(result,3);
    counter=0;
    sum=0;
    result = result_i * (gain_i/10) + (analogRead(EMG)/1023.0 * 3.3) * ((10-gain_i)/10);
  }else{
    sum+=analogRead(EMG)/1023.0 * 3.3;//3.3V換算
    counter++;
  }
}

void CloseHand(){
  Gripload_L = SerialServo.ReadLoad(5);
  Gripload_R = SerialServo.ReadLoad(4);
  
  if(Gripload_L < GripStrength && Grippos_L <= GripPosLimit)//L
  {//指を閉じる
    SerialServo.RegWritePosEx(5, 2047 - Grippos_L - 50 ,STS3032_v,STS3032_a);
    Grippos_L+=GripSpeed;
  }

  if( abs(Gripload_R) < GripStrength && Grippos_R <= GripPosLimit)//R
  {
    SerialServo.RegWritePosEx(4, 2047 + Grippos_R ,STS3032_v,STS3032_a);
    Grippos_R+=GripSpeed;
  }
}

void OpenHand(){
  Gripload_L = SerialServo.ReadLoad(5);
  Gripload_R = SerialServo.ReadLoad(4);
  
  if(Grippos_L >= GripDefaultPos)//L
  {//指を閉じる
    SerialServo.RegWritePosEx(5, 2047 - Grippos_L - 50 ,STS3032_v,STS3032_a);
    Grippos_L-=GripSpeed;
  }

  if(Grippos_R >= GripDefaultPos)//R
  {
    SerialServo.RegWritePosEx(4, 2047 + Grippos_R ,STS3032_v,STS3032_a);
    Grippos_R-=GripSpeed;
  }
}
