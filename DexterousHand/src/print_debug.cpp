#include <Arduino.h>
#include "print_debug.h"

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

void Serial_scan::receive(){
    if(Serial.available() > 0){
        buff = Serial.read();
    }
}

int Serial_scan::get(){
    return buff;
}

webserial::webserial(IMU_device &obj){
    p = &obj;
}

void webserial::WebSerialprint(){
    sensors_event_t event;
    p->getEvent(&event);

  /* Board layout:
         +----------+
         |         *| RST   PITCH  ROLL  HEADING
     ADR |*        *| SCL
     INT |*        *| SDA     ^            /->
     PS1 |*        *| GND     |            |
     PS0 |*        *| 3VO     Y    Z-->    \-X
         |         *| VIN
         +----------+
  */

  /* The WebSerial 3D Model Viewer expects data as heading, pitch, roll */
  Serial.print(F("Orientation: "));
  Serial.print(360 - (float)event.orientation.x);
  Serial.print(F(", "));
  Serial.print((float)event.orientation.y);
  Serial.print(F(", "));
  Serial.print((float)event.orientation.z);
  Serial.println(F(""));

  /* The WebSerial 3D Model Viewer also expects data as roll, pitch, heading */
  imu::Quaternion quat = p->getQuat();
  
  Serial.print(F("Quaternion: "));
  Serial.print((float)quat.w(), 4);
  Serial.print(F(", "));
  Serial.print((float)quat.x(), 4);
  Serial.print(F(", "));
  Serial.print((float)quat.y(), 4);
  Serial.print(F(", "));
  Serial.print((float)quat.z(), 4);
  Serial.println(F(""));

  /* Also send calibration data for each sensor. */
  uint8_t sys, gyro, accel, mag = 0;
  p->getCalibration(&sys, &gyro, &accel, &mag);
  Serial.print(F("Calibration: "));
  Serial.print(sys, DEC);
  Serial.print(F(", "));
  Serial.print(gyro, DEC);
  Serial.print(F(", "));
  Serial.print(accel, DEC);
  Serial.print(F(", "));
  Serial.print(mag, DEC);
  Serial.println(F(""));
}

