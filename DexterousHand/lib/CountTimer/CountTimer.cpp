#include "CountTimer.h"
#include <Arduino.h>

unsigned long CountTimer::buff[99][3];
int CountTimer::count_num;

CountTimer::CountTimer(){
    for (int i = 0; i < 99; i++) buff[i][0] = 0;
    count_num = 0;
}

bool CountTimer::wait(unsigned long* time){
    int index = search(time);//指定されたアドレスを持つ配列要素のインデックスを返す
    if(index == -1){//見つからないとき、新規作成
        set(time);
        return false;
    }else{
        refresh(index);
        if(buff[index][2]==true){
            remove(index);
            return true;
        }else{
            return false;
        }
    }
}

bool CountTimer::set(unsigned long* time){
    if(count_num < 99){
        count_num++;
        int index_new = search(0);
        buff[index_new][0] = (unsigned long)time;
        buff[index_new][1] = *time + millis();
        buff[index_new][2] = false;

        return true;
    }else{
        return false;
    }
    
}

int CountTimer::search(unsigned long* adress){
    int counter = 0;
    for(int i = 0 ; i < 99; i++){
        if(buff[i][0] == (unsigned long)adress){//指定されたアドレスの数値に要素が一致するか調べる
            return i;
        }else if(buff[i][0] != 0){//設定しているタイマーの個数を越えたら強制終了
            counter++;
            if(counter == count_num && adress != 0){
                return -1;
            }
        }
    }
    return -1;
}

bool CountTimer::remove(int index){
    if(count_num > 0){
        count_num--;
        buff[index][0] = 0;
        buff[index][1] = 0;
        buff[index][2] = false;

        return true;
    }else{
        return false;
    }

}

void CountTimer::refresh(int index){
    if(buff[index][1] <= millis()){//指定されたインデックスの終了時間が越えていたらステータスをtrueにする
        buff[index][2] = true;
    }
}