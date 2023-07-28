#include "Timer.h"
#include "Arduino.h"
void MyTimer::StartTickTime(long TickTime)
{
    this -> TickTime = TickTime;
    if(!(this -> OnTick))
    {
       start_Time = millis();
       this -> OnTick = true;
    }
}
void MyTimer::TickStop()
{
    this -> OnTick = false;
}
bool MyTimer::IsTimeOut()
{
    long TimeNow = millis();
    if ((TimeNow - start_Time) >= TickTime)
    {
        this -> OnTick = false;
        return true;
    }
    else return false;
}
