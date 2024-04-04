#include <Arduino.h>
#include "emg_calib.h"

EMG_CALIB emg_1(Serial,1);//Serial

void setup(){
    Serial.begin(115200);
    while(!Serial) delay(100);//シリアル通信の確立まで待機（Arduino純正ボード以外では必須)
}
void loop(){
    code1();

    
}

void code1(){
    float sample;
    bool result;
    
    //sampleに測定結果を格納
    emg_1.sampling(sample);
    Serial.println(sample);

    //resultに判定結果を格納
    //threshold_calibを1回も実行してないときは、筋電位センサの信号が
    //2.5vを超えているかで判定される
    emg_1.judge(result);
    Serial.println(result);

    delay(500);
    
}

void code2(){
    float sample,x;
    bool result;

    //最大値の7割を閾値として、生成された閾値をxに格納
    emg_1.threshold_calib(x,7);
    delay(1000);

    //設定した閾値で判定
    emg_1.judge(result);
    Serial.println(result);
    delay(1000);

    
}