#ifndef _IMU_H_
#define _IMU_H_

#define ROM 100
#define BUFF 200
#define addressForimu 0

#include <Adafruit_BNO055.h>
#include <stdint.h>
#include <EEPROM.h>
#include "address_map.h"


/*
Adafrut_BiNO055のクラスを継承して、追加の機能を付けた。
センサーオフセットの取得、EEPROMへの書込み、読み込み、設定
setupでEEPROMの初期化が必要
*/

class IMU_device : public Adafruit_BNO055{
public:
    using Adafruit_BNO055::Adafruit_BNO055;//コンストラクタは基底クラスのを参照
    void Load_calib();//
    int  Save_calib();//センサーオフセットを取得して、EEPROMに書込み
    void get_calib(uint8_t* buff);//セーブしているセンサーオフセットを出力

private:
    uint8_t calib_buff_R[22];
    uint8_t calib_buff_S[22];
};

#endif