#include"Countdown.h"



bool Time::CountDown(int second){

    std::time_t now_time;
    std::time_t change_time;
    std::time_t final_time;
    std::time_t begin_time;


    time(&now_time);
    change_time = now_time;
    begin_time = now_time;
    final_time = now_time + seconds;


    do{
        time(&now_time);
        if(int(now_time)>int(change_time)){
            std::cout<<now_time-begin_time<<std::endl;
            Sleep(500);
            system("cls");
            change_time = now_time;
        }
    }while(now_time<=final_time);

    if(now_time>=final_time)return true;
}