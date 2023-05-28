#ifndef _EMG_CALIB_
#define _EMG_CALIB_
#include <Arduino.h>
#include <MsTimer2.h>
#include <SoftwareSerial.h>

#define EMG 1


class EMG_CALIB{
    public:
    EMG_CALIB(SoftwareSerial& s);
    void threshold_calib(float& j1to2, float& j2to3);
    void sampling(float& x);

    private:
    SoftwareSerial* softSerial;
};


#endif