#ifndef _PRINT_DEBUG_
#define _PRINT_DEBUG_
#define END true

#include <Arduino.h>
#include "imu.h"

//Serial1はシリアルサーボとの通信に使うので、Serialをデバック用にする
//Serial.begin()の引数でserial1のピンを変更する
//通信にteratermとか必要かも
//プリントデバック用関数　S_print(変数名の文字列,変数名or配列名,(改行するなら)END);
//S_print(出力したいもの)でSerial.print()の代用も可能

template<class S,class T>
void S_print(S tag, const T num, bool n=false);

template<class S,class T>
void S_print(S tag, const T* vec, int s);

template<class S>
void S_print(const S tag, bool n = false);


class Serial_scan{
    public:
        Serial_scan():buff(0){}

        void receive();

        int get();

    private:

        int buff;

};

class webserial{
    public:

        webserial(IMU_device &obj);
        void WebSerialprint();
    
    private:

        IMU_device *p;
};



#endif