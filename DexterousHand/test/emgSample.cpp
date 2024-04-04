#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>//IMUのライブラリ

#include <Arduino.h>

#include "calculate.h"

Adafruit_BNO055 IMU(55,0x29);//IMUを扱うインスタンスを生成

EulerOrder axisOrder = EulerOrder::XYZ;//関節の座標系指定

double aim_qua_w=1;//目標クォータニオンの各要素
double aim_qua_x=0;
double aim_qua_y=0;
double aim_qua_z=0;
imu::Quaternion qua_aim(aim_qua_w,aim_qua_x,aim_qua_y,aim_qua_z);//目標クォータニオン生成

void setup(){

    Serial2.begin(115200);

    while(!Serial2) delay(100);//シリアル通信の確立まで待機（Arduino純正ボード以外では必須）

    bool imu_conection = IMU.begin();//IMUとの接続を取得
    if(!imu_conection){
        Serial2.println("no BNO055 detected");//接続に失敗していたら待機
        while(1){}
    }else{
        IMU.setExtCrystalUse(true);//IMU内部の水晶振動子を使うかどうか（このコピペでいい）
        Serial2.println("IMU_arm setting complete");//接続に成功
    }


        uint8_t mag_calib;//IMUのキャリブレーションの数値格納用
        uint8_t gyro_calib;
        uint8_t sys_calib;
        uint8_t accel_calib;
        
        do//IMUのコンパスの初期化（これを待たずに制御始めると途中で角度が大きく変動して暴走）
        {   //ジャイロ、加速度、コンパスのキャリブレーションの状態（１が最低３が最高）を取得
            IMU.getCalibration(&sys_calib,&gyro_calib,&accel_calib,&mag_calib);
            Serial2.print("calibrating.........");
            Serial2.print(mag_calib);
            Serial2.println("of3");
        } while (mag_calib != 3);//コンパスのキャリブレーションが最高になるまでループ回して待機

        IMU.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P3);//IMUの軸、座標設定
        IMU.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P3);//これについてはBNO055のデーターシート読んでほしい
}

void loop(){

    //シリアル通信の確立まで待機（Arduino純正ボード以外では必須）
    while(!Serial2) delay(100);

    //BNO055からクォータニオンで姿勢を取得
    imu::Quaternion qua_measured = IMU.getQuat();

    //qua_aimに対するqua_measuredの差分を求める
    imu::Quaternion qua_diff = diffQuaterniopn(qua_aim, qua_measured);

    //qua_diffをXYZの軸構成のオイラー角に変換する
    imu::Vector<3> euler = convertEuler(qua_diff,EulerOrder::XYZ);


    //オイラー角の各要素を出力
    //実用的にはモーターの角度に指定する
    //オイラー角の軸構成によらずに.x()のメゾットはｘ要素、.y()のメゾットはy要素という風にアクセスできる
    Serial2.print("X: ");
    Serial2.print(euler.x());

    Serial2.print("Y: ");
    Serial2.print(euler.y());

    Serial2.print("Z: ");
    Serial2.println(euler.z());
}