#ifndef _COUNTTIMER_
#define _COUNTTIMER_

class CountTimer{
    public:
        CountTimer();

        bool wait(unsigned long* time);//タイマーの時間をポインターで受け、新規のタイマーセットか時間経過がどれくらいかをboolで返す

        void refresh(int index);//配列を更新する。終了予定時刻と現在時刻を照合する

        bool remove(int index);//指定されたアドレスのタイマーを消去する。trueを返した後にwaitに呼び出される

        bool set(unsigned long* time);//新規にタイマーをセットする

        int  search(unsigned long* adress);//指定されたアドレスを持つ配列要素を検索する

    private:
        static unsigned long buff[99][3];//99個までタイマー管理可能　[呼び出し元アドレス][終了時間][戻り値]

        static int count_num;
};



#endif