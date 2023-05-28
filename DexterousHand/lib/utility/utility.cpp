#include "utility.h"

int RadToStep(double rad){
  float x = (rad/(2 * 3.14)) * 4095;
  return (int)x;
}

int move_time(int Pos,int prePos, int v, int a){
    return ((Pos - prePos)/v) * 1000 + (v/(a*100)) * 1000;
}

int StepRange_con(int raw){//step
    return raw-2047;
}

int StepRange_res(int raw){
  return raw + 2027;
}

double RadRange_con(double raw){
  if(raw <= 3.14) return raw;
  else return (raw - 3.14) - 3.14;
}

void wait(double time){
    
}