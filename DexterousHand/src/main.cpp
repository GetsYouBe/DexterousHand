#include <Arduino.h>
//#include <VarSpeedServo.h>
#include <Servo.h>
//サーボ系ライブラリは競合する

#include <MsTimer2.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


//角速度変換用 https://sgrsn1711.hatenablog.com/entry/2018/03/15/234155 より　y[deg/sec]=ax[speed]+b として
#define a 2.2809
#define b 0.3824

#define Interrupt_cycle 170//[ms]
#define noise -0.0001

#define MYO_WARE_PIN 0

/*加速度クラス
class angular_velocity{
  private:
    float x;
  public:
    angular_velocity(){
      x=0;
    }
    
    float speed(){
      return (x-b)/a;
    }

    float set(float prex,float latestx){
      x = (latestx-prex)/Interrupt_cycle;
    }

    float get(){
      return x;
    }
};*/

//インスタンス生成
Adafruit_BNO055 IMU_end = Adafruit_BNO055(1,0x28);//id address
Adafruit_BNO055 IMU_root = Adafruit_BNO055(2,0x29);//id address

volatile bool interrupt=false;//割り込み環境変数

//筋電位
int emg_counter=0;
double emg_result=0;
double emg_sum=0;
const float emg_loop_count=10.0;
int emg_judge = 4;
bool already_get_angle = false;
bool EMG;
bool test_flag = true;

Servo servo_x;
Servo servo_y;
Servo servo_z;
Servo servo_hand;

double x_angle=0;//オイラー角として使うために値域が-90<=0<=90
double y_angle=0;
double z_angle=0;

imu::Quaternion Target_quat;

imu::Vector<3> vec_ex = {1,0,0};//基本ベクトル
imu::Vector<3> vec_ey = {0,1,0};
imu::Vector<3> vec_ez = {0,0,1};

void setup() {
  Serial.begin(115200);Serial.println("start");

  pinMode(13, OUTPUT);
  
  bool end_conection = IMU_end.begin();
  bool root_conection = IMU_root.begin();
  /*IMUの設定--------------------------------------------------------*/
  if(!end_conection){//begin()はIMUとの通信開始
    Serial.println("no BNO055(end) detected");
  }else{
    IMU_end.setExtCrystalUse(true);
    Serial.println("IMU_end setting complete");
  }
  

  if(!root_conection){//begin()はIMUとの通信開始
    Serial.println("no BNO055(root) detected");
  }else{
    IMU_root.setExtCrystalUse(true);
    Serial.println("IMU_root setting complete");
  }

  if(!end_conection || !root_conection){//どちらかが見つからなかったら停止
    Serial.println("system stop");
    while(1);
  }
 

  /*サーボ初期化-----------------------------------------------------*/
  servo_x.attach(3);
  servo_y.attach(2);
  servo_z.attach(4);
  servo_hand.attach(5);
  servo_hand.write(0);

  /*割り込みタイマーの設定--------------------------------------------*/
  MsTimer2::set(Interrupt_cycle, Interrupt);
  MsTimer2::start();
}

void Interrupt() {
  //割り込みの確認用　筋電のと合わせるときに要注意
  static bool output = HIGH;
  digitalWrite(13, output);
  output = !output;

  Serial.println("interrupt");
  interrupt=true;
}

void loop() {
  if(interrupt){//割り込み処理
    interrupt = false;

    double preInput_end[4];
    double preInput_root[4];

    //平均用
    double sum_end[4]={0,0,0,0};
    double sum_root[4]={0,0,0,0};
  
    //代入回数カウント
    int count_end[4]={0,0,0,0};
    int count_root[4]={0,0,0,0};
    
    //ノイズ除去
    for(int j=0;j<20;j++){

      imu::Quaternion pre_quat_end = IMU_end.getQuat();//先端のIMUから取得
      preInput_end[0] = pre_quat_end.w();
      preInput_end[1] = pre_quat_end.x();
      preInput_end[2] = pre_quat_end.y();
      preInput_end[3] = pre_quat_end.z();

      imu::Quaternion pre_quat_root = IMU_root.getQuat();//根本のIMUから取得
      preInput_root[0] = pre_quat_root.w();
      preInput_root[1] = pre_quat_root.x();
      preInput_root[2] = pre_quat_root.y();
      preInput_root[3] = pre_quat_root.z();

      for(int i=0;i<4;i++){
        if(preInput_end[i] != noise){
          sum_end[i] += preInput_end[i];
          count_end[i]++;
        }
        if(preInput_root[i] != noise){
          sum_root[i] += preInput_root[i];
          count_root[i]++;
        }
      }
    }
    
    //メインのクォータニオン格納
    imu::Quaternion quat_end(
      sum_end[0] / count_end[0],
      sum_end[1] / count_end[1],
      sum_end[2] / count_end[2],
      sum_end[3] / count_end[3]
    );

    imu::Quaternion quat_root(
      sum_root[0] / count_root[0],
      sum_root[1] / count_root[1],
      sum_root[2] / count_root[2],
      sum_root[3] / count_root[3]
    );
    
    //myo ware
    if(emg_counter>=emg_loop_count){//測定値を積分か平均をとる
    emg_result=emg_sum/emg_loop_count;
    //Serial.println(result,3);
    emg_counter=0;
    emg_sum=0;
    }else{
    emg_sum+=analogRead(MYO_WARE_PIN)/1023.0 * 5.0;//5V換算
    emg_counter++;
    }
    if(emg_result>=emg_judge){//筋電の信号確認用　IMUのほうでは割り込み確認で使っているので要注意
      EMG = true;
    }else{
      EMG = false;
    }
    /*
    Serial.print("qW: ");
    Serial.print(quat_root.w(), 4);
    Serial.print(" qX: ");
    Serial.print(quat_root.x(), 4);
    Serial.print(" qY: ");
    Serial.print(quat_root.y(), 4);
    Serial.print(" qZ: ");
    Serial.print(quat_root.z(), 4);
    Serial.println("\t\t");*/

    if(EMG || test_flag){//力が入っているときの処理

      servo_hand.write(30);

      quat_root.normalize();//計算に使うために正規化
      quat_end.normalize();

      if(!already_get_angle){//目標角を取得していない
        Serial.println("get angle");
        //目標角取得：アームの状態をオイラー角から分解して、四元数に変換：根本のものに適用：先端の四元数が求まる
        

        imu::Quaternion motor_x;//オイラー角それぞれに対応するクォータニオン
        imu::Quaternion motor_y;
        imu::Quaternion motor_z;

        motor_x.fromAxisAngle(vec_ex,x_angle);//サーボの角度からクォータニオンの中身を生成
        motor_y.fromAxisAngle(vec_ey,y_angle);
        motor_z.fromAxisAngle(vec_ez,z_angle);

        Target_quat = quat_root * motor_x * motor_y * motor_z;//先端のクォータニオンを取得 x-y-z系のオイラー角として計算

        Target_quat.normalize();//目標クォータニオンを正規化　いらないかも
        already_get_angle = true;
      }

      //差分を求める：オイラー角に変換：角度の値域を修正してからサーボに渡す ad = b より、d = a^-1 b

      imu::Quaternion diff = quat_root.conjugate() * Target_quat;//差分を求める

      imu::Vector<3> motor_euler = diff.toEuler();

      motor_euler.toDegrees();

      
      Serial.print("x: ");
      Serial.print(motor_euler.x(), 4);
      Serial.print(" y: ");
      Serial.print(motor_euler.y(), 4);
      Serial.print(" z: ");
      Serial.print(motor_euler.z(), 4);
      Serial.println("\t\t");

      servo_x.write(motor_euler.x()+90);
      servo_y.write(motor_euler.y()+90);
      servo_z.write(motor_euler.z()+90);
    }else{
      servo_hand.write(0);
    }
  }
}
