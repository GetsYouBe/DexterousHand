#ifndef _EMG_CALIB_
#define _EMG_CALIB_
#include <Arduino.h>

#define EMG 1


class EMG_CALIB{
    public:
    EMG_CALIB(HardwareSerial& s);
    void threshold_calib(float& j1to2, float& j2to3, float& j1to3, float& n12, float& n23, float& n13);
    void sampling(float& x);

    private:
    HardwareSerial* softSerial;
};


#endif