#include "utility.h"
#include <Arduino.h>


int RadToStep(double rad){//ラジアンをモーターのステップ数に変換する。負の値を入力すると負のステップ数になる
  float x = (rad/(2 * 3.14)) * 4095;
  return (int)x;
}

int move_time(int Pos,int prePos, int v, int a){
    return (float)abs(Pos - prePos)/v * 1000 + (float)v/(a*100) * 1000;
}

int StepRange_con(int step){//step 0 < raw < 4095 -> -2027 < raw < 2027
    return step-2047;
}

int StepRange_res(int step){//-2027 < raw < 2027 -> 0 < return < 4095
  return step + 2027;
}

double RadRange_con(double rad){//rawがpie以上の時に反時計回りに一周させて、ふごうをマイナスにする
  if(rad <= 3.14) return rad;
  else return (rad - 3.14) - 3.14;
}