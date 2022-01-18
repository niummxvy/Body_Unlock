#include<iostream>
#include<ctime>
#include<windows.h>
class Time{
    private:
        int seconds;
    public:
        Time();
        bool CountDown(int seconds);
};