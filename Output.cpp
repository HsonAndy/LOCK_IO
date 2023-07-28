#include "Timer.h"
#include "Output.h"
#include "Arduino.h"
#include "DFRobot_MCP23017.h"

void MyOutput::Set_toggle(bool value)
{
   this -> flag_toogle = value;
   this -> Set_State(this -> State);
}
void MyOutput::Init(int PIN_Num)
{
   this -> PIN_NUM = PIN_Num ;
   if(PIN_Num != -1)pinMode(PIN_Num, OUTPUT);
   if(PIN_Num != -1)digitalWrite(PIN_Num, this -> GetLogic(false));
}
void MyOutput::Init(int PIN_Num ,DFRobot_MCP23017& mcp)
{
   this -> _mcp = &mcp;
   this -> PIN_NUM = PIN_Num ;
   if(PIN_Num == -1)return;
   this -> flag_mcp = true;
   if(PIN_Num == 0) _mcp -> pinMode(_mcp->eGPA0 , OUTPUT);
   else if(PIN_Num == 1) _mcp -> pinMode(_mcp->eGPA1 , OUTPUT);
   else if(PIN_Num == 2) _mcp -> pinMode(_mcp->eGPA2 , OUTPUT);
   else if(PIN_Num == 3) _mcp -> pinMode(_mcp->eGPA3 , OUTPUT);
   else if(PIN_Num == 4) _mcp -> pinMode(_mcp->eGPA4 , OUTPUT);
   else if(PIN_Num == 5) _mcp -> pinMode(_mcp->eGPA5 , OUTPUT);
   else if(PIN_Num == 6) _mcp -> pinMode(_mcp->eGPA6 , OUTPUT);
   else if(PIN_Num == 7) _mcp -> pinMode(_mcp->eGPA7 , OUTPUT);
   else if(PIN_Num == 8) _mcp -> pinMode(_mcp->eGPB0 , OUTPUT);
   else if(PIN_Num == 9) _mcp -> pinMode(_mcp->eGPB1 , OUTPUT);
   else if(PIN_Num == 10) _mcp -> pinMode(_mcp->eGPB2 , OUTPUT);
   else if(PIN_Num == 11) _mcp -> pinMode(_mcp->eGPB3 , OUTPUT);
   else if(PIN_Num == 12) _mcp -> pinMode(_mcp->eGPB4 , OUTPUT);
   else if(PIN_Num == 13) _mcp -> pinMode(_mcp->eGPB5 , OUTPUT);
   else if(PIN_Num == 14) _mcp -> pinMode(_mcp->eGPB6 , OUTPUT);
   else if(PIN_Num == 15) _mcp -> pinMode(_mcp->eGPB7 , OUTPUT);
}
void MyOutput::Init(int PIN_Num , bool flag_toogle)
{
   this -> flag_toogle = flag_toogle;
   this -> PIN_NUM = PIN_Num ;
   if(PIN_Num != -1)pinMode(PIN_Num, OUTPUT);
   if(PIN_Num != -1)digitalWrite(PIN_Num, this -> GetLogic(false));
}
void MyOutput::Set_State(bool ON_OFF)
{
    State = ON_OFF;
    if(this -> PIN_NUM == -1) return;
    if(this -> flag_mcp) 
    {
       if(ON_OFF) State_ON = true;
       else State_OFF = true;
    }
    else
    {
       digitalWrite(this -> PIN_NUM, this -> GetLogic(ON_OFF));
    }
}

void MyOutput::Blink(int Time)
{
  this -> OnDelayTime = Time;
  MyOutput::Blink();
}
bool MyOutput::GetLogic(bool value)
{
  if(this -> flag_toogle == false)
  {
     return value;
  }
  else
  {
    return !value;
  }
}
void MyOutput::Blink()
{
  int PIN = this -> PIN_NUM;

  if( (this -> cnt) == 254)
  {
    State = false;
    if(PIN != -1)
    {
       if(this -> flag_mcp)
       {
         if(this -> PIN_NUM == 0) _mcp -> digitalWrite(_mcp->eGPA0 , this -> GetLogic(false));
         else if(this -> PIN_NUM == 1) _mcp -> digitalWrite(_mcp->eGPA1 , this -> GetLogic(false));
         else if(this -> PIN_NUM == 2) _mcp -> digitalWrite(_mcp->eGPA2 , this -> GetLogic(false));
         else if(this -> PIN_NUM == 3) _mcp -> digitalWrite(_mcp->eGPA3 , this -> GetLogic(false));
         else if(this -> PIN_NUM == 4) _mcp -> digitalWrite(_mcp->eGPA4 , this -> GetLogic(false));
         else if(this -> PIN_NUM == 5) _mcp -> digitalWrite(_mcp->eGPA5 , this -> GetLogic(false));
         else if(this -> PIN_NUM == 6) _mcp -> digitalWrite(_mcp->eGPA6 , this -> GetLogic(false));
         else if(this -> PIN_NUM == 7) _mcp -> digitalWrite(_mcp->eGPA7 , this -> GetLogic(false));
         else if(this -> PIN_NUM == 8) _mcp -> digitalWrite(_mcp->eGPB0 , this -> GetLogic(false));
         else if(this -> PIN_NUM == 9) _mcp -> digitalWrite(_mcp->eGPB1 , this -> GetLogic(false));
         else if(this -> PIN_NUM == 10) _mcp -> digitalWrite(_mcp->eGPB2 , this -> GetLogic(false));
         else if(this -> PIN_NUM == 11) _mcp -> digitalWrite(_mcp->eGPB3 , this -> GetLogic(false));
         else if(this -> PIN_NUM == 12) _mcp -> digitalWrite(_mcp->eGPB4 , this -> GetLogic(false));
         else if(this -> PIN_NUM == 13) _mcp -> digitalWrite(_mcp->eGPB5 , this -> GetLogic(false));
         else if(this -> PIN_NUM == 14) _mcp -> digitalWrite(_mcp->eGPB6 , this -> GetLogic(false));
         else if(this -> PIN_NUM == 15) _mcp -> digitalWrite(_mcp->eGPB7 , this -> GetLogic(false));
       }
       else
       {
         digitalWrite(PIN, this -> GetLogic(false));
       }
       
    }
    (this -> cnt) = 255 ;
  }
  if( this -> OnDelayTime == 1 || State_ON ) 
  {
    State = true;
    State_ON = false;
    if(PIN != -1)
    {
       if(this -> flag_mcp)
       {
         if(this -> PIN_NUM == 0) _mcp -> digitalWrite(_mcp->eGPA0 , this -> GetLogic(true));
         else if(this -> PIN_NUM == 1) _mcp -> digitalWrite(_mcp->eGPA1 , this -> GetLogic(true));
         else if(this -> PIN_NUM == 2) _mcp -> digitalWrite(_mcp->eGPA2 , this -> GetLogic(true));
         else if(this -> PIN_NUM == 3) _mcp -> digitalWrite(_mcp->eGPA3 , this -> GetLogic(true));
         else if(this -> PIN_NUM == 4) _mcp -> digitalWrite(_mcp->eGPA4 , this -> GetLogic(true));
         else if(this -> PIN_NUM == 5) _mcp -> digitalWrite(_mcp->eGPA5 , this -> GetLogic(true));
         else if(this -> PIN_NUM == 6) _mcp -> digitalWrite(_mcp->eGPA6 , this -> GetLogic(true));
         else if(this -> PIN_NUM == 7) _mcp -> digitalWrite(_mcp->eGPA7 , this -> GetLogic(true));
         else if(this -> PIN_NUM == 8) _mcp -> digitalWrite(_mcp->eGPB0 , this -> GetLogic(true));
         else if(this -> PIN_NUM == 9) _mcp -> digitalWrite(_mcp->eGPB1 , this -> GetLogic(true));
         else if(this -> PIN_NUM == 10) _mcp -> digitalWrite(_mcp->eGPB2 , this -> GetLogic(true));
         else if(this -> PIN_NUM == 11) _mcp -> digitalWrite(_mcp->eGPB3 , this -> GetLogic(true));
         else if(this -> PIN_NUM == 12) _mcp -> digitalWrite(_mcp->eGPB4 , this -> GetLogic(true));
         else if(this -> PIN_NUM == 13) _mcp -> digitalWrite(_mcp->eGPB5 , this -> GetLogic(true));
         else if(this -> PIN_NUM == 14) _mcp -> digitalWrite(_mcp->eGPB6 , this -> GetLogic(true));
         else if(this -> PIN_NUM == 15) _mcp -> digitalWrite(_mcp->eGPB7 , this -> GetLogic(true));
       }
       else
       {
         digitalWrite(PIN, this -> GetLogic(true));
       }
       
    }
    if(Output_ON != nullptr) Output_ON();
    this -> OnDelayTime_buf = OnDelayTime;    
    return;
  }
  if( this -> OnDelayTime == 0 || State_OFF) 
  {
    State = false;
    State_OFF = false;
    if(PIN != -1)
    {
       if(this -> flag_mcp)
       {
         if(this -> PIN_NUM == 0) _mcp -> digitalWrite(_mcp->eGPA0 , this -> GetLogic(false));
         else if(this -> PIN_NUM == 1) _mcp -> digitalWrite(_mcp->eGPA1 , this -> GetLogic(false));
         else if(this -> PIN_NUM == 2) _mcp -> digitalWrite(_mcp->eGPA2 , this -> GetLogic(false));
         else if(this -> PIN_NUM == 3) _mcp -> digitalWrite(_mcp->eGPA3 , this -> GetLogic(false));
         else if(this -> PIN_NUM == 4) _mcp -> digitalWrite(_mcp->eGPA4 , this -> GetLogic(false));
         else if(this -> PIN_NUM == 5) _mcp -> digitalWrite(_mcp->eGPA5 , this -> GetLogic(false));
         else if(this -> PIN_NUM == 6) _mcp -> digitalWrite(_mcp->eGPA6 , this -> GetLogic(false));
         else if(this -> PIN_NUM == 7) _mcp -> digitalWrite(_mcp->eGPA7 , this -> GetLogic(false));
         else if(this -> PIN_NUM == 8) _mcp -> digitalWrite(_mcp->eGPB0 , this -> GetLogic(false));
         else if(this -> PIN_NUM == 9) _mcp -> digitalWrite(_mcp->eGPB1 , this -> GetLogic(false));
         else if(this -> PIN_NUM == 10) _mcp -> digitalWrite(_mcp->eGPB2 , this -> GetLogic(false));
         else if(this -> PIN_NUM == 11) _mcp -> digitalWrite(_mcp->eGPB3 , this -> GetLogic(false));
         else if(this -> PIN_NUM == 12) _mcp -> digitalWrite(_mcp->eGPB4 , this -> GetLogic(false));
         else if(this -> PIN_NUM == 13) _mcp -> digitalWrite(_mcp->eGPB5 , this -> GetLogic(false));
         else if(this -> PIN_NUM == 14) _mcp -> digitalWrite(_mcp->eGPB6 , this -> GetLogic(false));
         else if(this -> PIN_NUM == 15) _mcp -> digitalWrite(_mcp->eGPB7 , this -> GetLogic(false));
       }
       else
       {
         digitalWrite(PIN, this -> GetLogic(false));
       }
       
    }
    if(Output_OFF != nullptr) Output_OFF();
    this -> OnDelayTime_buf = OnDelayTime;   
    return;
  } 
  if( (this -> cnt) == 255)
  {
     Trigger = false;
     (this -> cnt) = 1;
  }
  if( (this -> cnt) == 1)
  {    
    if(BlinkEnable == true) Trigger = true;
    if(Trigger) 
    {
       (this -> cnt) = (this -> cnt) + 1 ;
    }
  }
  if( (this -> cnt) == 2)
  {    
    State = true;
    if(PIN != -1)
    {
       if(this -> flag_mcp)
       {
         if(this -> PIN_NUM == 0) _mcp -> digitalWrite(_mcp->eGPA0 , this -> GetLogic(true));
         else if(this -> PIN_NUM == 1) _mcp -> digitalWrite(_mcp->eGPA1 , this -> GetLogic(true));
         else if(this -> PIN_NUM == 2) _mcp -> digitalWrite(_mcp->eGPA2 , this -> GetLogic(true));
         else if(this -> PIN_NUM == 3) _mcp -> digitalWrite(_mcp->eGPA3 , this -> GetLogic(true));
         else if(this -> PIN_NUM == 4) _mcp -> digitalWrite(_mcp->eGPA4 , this -> GetLogic(true));
         else if(this -> PIN_NUM == 5) _mcp -> digitalWrite(_mcp->eGPA5 , this -> GetLogic(true));
         else if(this -> PIN_NUM == 6) _mcp -> digitalWrite(_mcp->eGPA6 , this -> GetLogic(true));
         else if(this -> PIN_NUM == 7) _mcp -> digitalWrite(_mcp->eGPA7 , this -> GetLogic(true));
         else if(this -> PIN_NUM == 8) _mcp -> digitalWrite(_mcp->eGPB0 , this -> GetLogic(true));
         else if(this -> PIN_NUM == 9) _mcp -> digitalWrite(_mcp->eGPB1 , this -> GetLogic(true));
         else if(this -> PIN_NUM == 10) _mcp -> digitalWrite(_mcp->eGPB2 , this -> GetLogic(true));
         else if(this -> PIN_NUM == 11) _mcp -> digitalWrite(_mcp->eGPB3 , this -> GetLogic(true));
         else if(this -> PIN_NUM == 12) _mcp -> digitalWrite(_mcp->eGPB4 , this -> GetLogic(true));
         else if(this -> PIN_NUM == 13) _mcp -> digitalWrite(_mcp->eGPB5 , this -> GetLogic(true));
         else if(this -> PIN_NUM == 14) _mcp -> digitalWrite(_mcp->eGPB6 , this -> GetLogic(true));
         else if(this -> PIN_NUM == 15) _mcp -> digitalWrite(_mcp->eGPB7 , this -> GetLogic(true));
       }
       else
       {
         digitalWrite(PIN, this -> GetLogic(true));
       }
       
    }
    if(Output_ON != nullptr) Output_ON();
    //printf("Output ON PIN : %d , OnDelayTime : %d\n",PIN,OnDelayTime);
    myTimer.StartTickTime(this -> OnDelayTime);
    (this -> cnt) = (this -> cnt) + 1 ;
  }
  if( (this -> cnt) == 3)
  {
    if(myTimer.IsTimeOut())
    {
      State = false;
      if(PIN != -1)
      {
         if(this -> flag_mcp)
         {
           if(this -> PIN_NUM == 0) _mcp -> digitalWrite(_mcp->eGPA0 , this -> GetLogic(false));
           else if(this -> PIN_NUM == 1) _mcp -> digitalWrite(_mcp->eGPA1 , this -> GetLogic(false));
           else if(this -> PIN_NUM == 2) _mcp -> digitalWrite(_mcp->eGPA2 , this -> GetLogic(false));
           else if(this -> PIN_NUM == 3) _mcp -> digitalWrite(_mcp->eGPA3 , this -> GetLogic(false));
           else if(this -> PIN_NUM == 4) _mcp -> digitalWrite(_mcp->eGPA4 , this -> GetLogic(false));
           else if(this -> PIN_NUM == 5) _mcp -> digitalWrite(_mcp->eGPA5 , this -> GetLogic(false));
           else if(this -> PIN_NUM == 6) _mcp -> digitalWrite(_mcp->eGPA6 , this -> GetLogic(false));
           else if(this -> PIN_NUM == 7) _mcp -> digitalWrite(_mcp->eGPA7 , this -> GetLogic(false));
           else if(this -> PIN_NUM == 8) _mcp -> digitalWrite(_mcp->eGPB0 , this -> GetLogic(false));
           else if(this -> PIN_NUM == 9) _mcp -> digitalWrite(_mcp->eGPB1 , this -> GetLogic(false));
           else if(this -> PIN_NUM == 10) _mcp -> digitalWrite(_mcp->eGPB2 , this -> GetLogic(false));
           else if(this -> PIN_NUM == 11) _mcp -> digitalWrite(_mcp->eGPB3 , this -> GetLogic(false));
           else if(this -> PIN_NUM == 12) _mcp -> digitalWrite(_mcp->eGPB4 , this -> GetLogic(false));
           else if(this -> PIN_NUM == 13) _mcp -> digitalWrite(_mcp->eGPB5 , this -> GetLogic(false));
           else if(this -> PIN_NUM == 14) _mcp -> digitalWrite(_mcp->eGPB6 , this -> GetLogic(false));
           else if(this -> PIN_NUM == 15) _mcp -> digitalWrite(_mcp->eGPB7 , this -> GetLogic(false));
         }
         else
         {
           digitalWrite(PIN, this -> GetLogic(false));
         }
         
      }
      if(Output_OFF != nullptr) Output_OFF();
      myTimer.StartTickTime(this -> OnDelayTime);
      //printf("Output OFF PIN : %d , OnDelayTime : %d\n",PIN,OnDelayTime);
      (this -> cnt) = (this -> cnt) + 1 ;
    }
  }
  if( (this -> cnt) == 4)
  {
    if(myTimer.IsTimeOut())
    {      
      (this -> cnt) = 255 ;
    }
  }
}
