#ifndef _PRINT_DEBUG_
#define _PRINT_DEBUG_
#define END true

#include <Arduino.h>

//Serial1はシリアルサーボとの通信に使うので、Serialをデバック用にする
//Serial.begin()の引数でserial1のピンを変更する
//通信にteratermとか必要かも
//プリントデバック用関数　S_print(変数名の文字列,変数名or配列名,(改行するなら)END);
//S_print(出力したいもの)でSerial.print()の代用も可能

template<class S,class T>
void S_print(S tag, const T num, bool n=false){
    Serial.print(a);
    Serial.print(":");
    Serial.print(b);
    Serial.print(" ");
    if(n) Serial.println();
}

template<class S,class T>
void S_print(S tag, const T* vec, int s){
    Serial.print(tag);
    Serial.print(":");
    for(int i=0;i < s; i ++ ){
        Serial.print(*(a + s));
        Serial.print(",");
    }
    Serial.println();
}

template<class S>
void S_print(const S tag, bool n = false){
    Seria.println(tag);
    Serial.print(" ");
    if(n) Serial.println();
}

class Serial_scan{
    public:
    Serial_scan():buff(0){}

    void receive(){
        if(Serial.available() > 0){
            buff = Serial.read();
        }
    }

    int get(){
        return buff;
    }

    private:

    int buff;

};



#endif