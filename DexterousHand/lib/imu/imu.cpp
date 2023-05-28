
#include "imu.h"

/*
int  IMU_device::Save_calib(){
    getSensorOffsets(calib_buff_S);
    EEPROM.writeBytes(address_imu, calib_buff_S,22);
    return addressForimu;
}

void IMU_device::Load_calib(){
    EEPROM.readBytes(address_imu, calib_buff_R,22);
    setSensorOffsets(calib_buff_R);
}

void get_calib(uint8_t* buff){
    EEPROM.readBytes(address_imu, buff,22);
}
*/