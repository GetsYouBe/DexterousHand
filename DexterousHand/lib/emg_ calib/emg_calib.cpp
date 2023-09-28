#include "emg_calib.h"


EMG_CALIB::EMG_CALIB(HardwareSerial& s){
    softSerial = &s;
}

void EMG_CALIB::sampling(float& x){
    for(int i=0;i<100;i++){
        x += analogRead(EMG)/1023.0 * 5.0;
        delay(10);
        softSerial->print("|");
    }

    x /= 100.0;
}

void EMG_CALIB::threshold_calib(float& j1to2, float& j2to3, float& j1to3, float& n12, float& n23, float& n13){
    float low=0;
    float mid=0;
    float high=0;
    
    softSerial->println("calibration start");
    softSerial->println("Please relax your arm.....");
    sampling(low);
    softSerial->flush();
    delay(1000);

    softSerial->println("Put a little pressure on your arm. ....");
    sampling(mid);
    delay(1000);

    softSerial->println("Put strong pressure on your arm.. ....");
    sampling(high);
    delay(1000);

    j1to2 = ( (10 - n12) * low + n12 * mid ) / 10.0;
    j2to3 = ( (10 - n23) * mid + n23 * high ) / 10.0;
    j1to3 = ( (10 - n13) * low + n13 * high ) / 10.0;
    
    softSerial->println("completed!");
    delay(1000);
}