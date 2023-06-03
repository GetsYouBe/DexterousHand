#ifndef _PRINT_DEBUG_
#define _PRINT_DEBUG_
#define END true

#include <Arduino.h>
#include <stdint.h>
#include <SoftwareSerial.h>

//Serial1はシリアルサーボとの通信に使うので、Serialをデバック用にする
//Serial.begin()の引数でserial1のピンを変更する
//通信にteratermとか必要かも

class Serial_debug{
    public:
        Serial_debug(HardwareSerial &serial);
        void receive(int &result);
        void WebSerialprint(float qw,float qx,float qy,float qz, uint8_t calib_sys, uint8_t calib_gyro, uint8_t calib_accel, uint8_t calib_mag);//シリアル通信でIMUのパラメータを出力
        void WebSerialprint(float ox,float oy,float oz, uint8_t calib_sys, uint8_t calib_gyro, uint8_t calib_accel, uint8_t calib_mag);
        void TimerDebug(String tag);
        String read_Serial();

    private:
        HardwareSerial* Serial_pointer;
        double timer;
        bool Timer_launch;

};


#endif