#include <Arduino.h>
#include "print_debug.h"
#include <SoftwareSerial.h>

Serial_debug::Serial_debug(SoftwareSerial& serial){//シリアルのインスタンスのポインターを受け取り、このクラス内からメンバ関数を使えるようにする
    Serial_pointer = &serial;
}

void Serial_debug::receive(int &result){
    if(Serial_pointer->available()){
      result = Serial_pointer->read();
    }
}

String Serial_debug::read_Serial(){

   if(Serial.available() > 0){
    String receive = Serial_pointer->readString();
    receive.trim();
    return receive;
   }

   return "nothing" ;
}

//adafruit のweb3Dviewerでオイラー角とクォータニオンを表示するための関数

void Serial_debug::WebSerialprint(float qw,float qx,float qy,float qz, uint8_t calib_sys, uint8_t calib_gyro, uint8_t calib_accel, uint8_t calib_mag){//クォータニオンとセンサーのキャリブレーションパラメータを引数にとり、adaruitのweb3Dviewerに対応するよう出力

  /* The WebSerial 3D Model Viewer also expects data as roll, pitch, heading */
  Serial_pointer->print(F("Quaternion: "));
  Serial_pointer->print((float)qw, 4);
  Serial_pointer->print(F(", "));
  Serial_pointer->print((float)qx, 4);
  Serial_pointer->print(F(", "));
  Serial_pointer->print((float)qy, 4);
  Serial_pointer->print(F(", "));
  Serial_pointer->print((float)qz, 4);
  Serial_pointer->println(F(""));

  /* Also send calibration data for each sensor. */
  Serial_pointer->print(F("Calibration: "));
  Serial_pointer->print(calib_sys, DEC);
  Serial_pointer->print(F(", "));
  Serial_pointer->print(calib_gyro, DEC);
  Serial_pointer->print(F(", "));
  Serial_pointer->print(calib_accel, DEC);
  Serial_pointer->print(F(", "));
  Serial_pointer->print(calib_mag, DEC);
  Serial_pointer->println(F(""));
}

void Serial_debug::WebSerialprint(float ox,float oy,float oz, uint8_t calib_sys, uint8_t calib_gyro, uint8_t calib_accel, uint8_t calib_mag){
  /* The WebSerial 3D Model Viewer expects data as heading, pitch, roll */
  Serial_pointer->print(F("Orientation: "));
  Serial_pointer->print(360 - (float)ox);
  Serial_pointer->print(F(", "));
  Serial_pointer->print((float)oy);
  Serial_pointer->print(F(", "));
  Serial_pointer->print((float)oz);
  Serial_pointer->print(F(""));


  /* Also send calibration data for each sensor. */
  Serial_pointer->print(F("Calibration: "));
  Serial_pointer->print(calib_sys, DEC);
  Serial_pointer->print(F(", "));
  Serial_pointer->print(calib_gyro, DEC);
  Serial_pointer->print(F(", "));
  Serial_pointer->print(calib_accel, DEC);
  Serial_pointer->print(F(", "));
  Serial_pointer->print(calib_mag, DEC);
  Serial_pointer->println(F(""));
}

