#include "Timer.h"
#include "LED.h"
#include "Arduino.h"

void MyLED::Init(int PIN_Num)
{
   this -> PIN_NUM = PIN_Num ;
   if(PIN_Num != -1)pinMode(PIN_Num, OUTPUT);
}
void MyLED::Blink(int Time)
{
  this -> BlinkTime = Time;
  MyLED::Blink();
}
void MyLED::Blink()
{
  int PIN = this -> PIN_NUM;
  if( this -> BlinkTime == 1 ) 
  {
    if(PIN != -1)digitalWrite(PIN, HIGH);
    if(LED_ON != nullptr) LED_ON();
    return;
  }
  if( this -> BlinkTime == 0 ) 
  {
    if(PIN != -1)digitalWrite(PIN, LOW);
    if(LED_OFF != nullptr) LED_OFF();
    return;
  }  
  if( (this -> cnt) == 1)
  {    
    if(PIN != -1)digitalWrite(PIN, HIGH);
    if(LED_ON != nullptr) LED_ON();
    myTimer.StartTickTime(this -> BlinkTime);
    (this -> cnt) = (this -> cnt) + 1 ;
  }
  if( (this -> cnt) == 2)
  {
    if(myTimer.IsTimeOut())
    {
      if(PIN != -1)digitalWrite(PIN, LOW);
      if(LED_OFF != nullptr) LED_OFF();
      myTimer.StartTickTime(this -> BlinkTime);
      (this -> cnt) = (this -> cnt) + 1 ;
    }
  }
  if( (this -> cnt) == 3)
  {
    if(myTimer.IsTimeOut())
    {
      (this -> cnt) = 1 ;
    }
  }
}
