#include "emg_calib.h"


EMG_CALIB::EMG_CALIB(HardwareSerial& s, int EMG_pin){
    softSerial = &s;
    pin = EMG_pin;
}

void EMG_CALIB::sampling(float& x){
    for(int i=0;i<100;i++){
        x += analogRead(pin)/1023.0 * 5.0;
        delay(10);
        softSerial->print("|");
        softSerial->flush();
    }

    x /= 100.0;
}

void EMG_CALIB::threshold_calib(float& j1to3, float n13){
    float low=0;
    //float mid=0;
    float high=0;
    
    softSerial->println("calibration start");
    softSerial->flush();
    softSerial->println("Please relax your arm.....");
    softSerial->flush();
    sampling(low);
    delay(1000);

    // softSerial->println("Put a little pressure on your arm. ....");
    // sampling(mid);
    // delay(1000);

    softSerial->println("Put strong pressure on your arm.. ....");
    softSerial->flush();
    sampling(high);
    delay(1000);

    // j1to2 = ( (10 - n12) * low + n12 * mid ) / 10.0;
    // j2to3 = ( (10 - n23) * mid + n23 * high ) / 10.0;
    
    judge_13 = ( (10 - n13) * low + n13 * high ) / 10.0;
    j1to3 = judge_13;
    
    softSerial->println("completed!");
    softSerial->flush();
    delay(1000);
}

void EMG_CALIB::judge(bool& result){
    float sample;
    sampling(sample);
    if(sample < judge_13){
        result = false;
    }else{
        result= true;
    }
}