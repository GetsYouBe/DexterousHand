#ifndef _EMG_CALIB_
#define _EMG_CALIB_
#include <Arduino.h>

struct threshold{
    float j1to2;
    float j2to3;
    float j1to3;
};

struct ratio{
    float n12;
    float n23;
    float n13;
};

class EMG_CALIB{
    public:
    EMG_CALIB(HardwareSerial& s,int EMG_pin);
    void threshold_calib(float& j1to3, float n13);
    void sampling(float& x);
    void judge(bool& result);

    private:
    HardwareSerial* softSerial;
    int pin;
    int judge_13 = 2.5;
};


#endif