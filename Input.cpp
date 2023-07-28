#include "Timer.h"
#include "Input.h"
#include "Arduino.h"

void MyInput::Init(int PIN_Num)
{
   this -> PIN_NUM = PIN_Num ;
   if(PIN_Num != -1)
   {
      pinMode(PIN_Num, INPUT_PULLUP);
   }
}
void MyInput::Init(int PIN_Num ,DFRobot_MCP23017& mcp)
{
   this -> _mcp = &mcp;
   this -> PIN_NUM = PIN_Num ;
   if(PIN_Num == -1)return;
   this -> flag_mcp = true;
   if(PIN_Num == 0) _mcp -> pinMode(_mcp->eGPA0 , INPUT_PULLUP);
   else if(PIN_Num == 1) _mcp -> pinMode(_mcp->eGPA1 , INPUT_PULLUP);
   else if(PIN_Num == 2) _mcp -> pinMode(_mcp->eGPA2 , INPUT_PULLUP);
   else if(PIN_Num == 3) _mcp -> pinMode(_mcp->eGPA3 , INPUT_PULLUP);
   else if(PIN_Num == 4) _mcp -> pinMode(_mcp->eGPA4 , INPUT_PULLUP);
   else if(PIN_Num == 5) _mcp -> pinMode(_mcp->eGPA5 , INPUT_PULLUP);
   else if(PIN_Num == 6) _mcp -> pinMode(_mcp->eGPA6 , INPUT_PULLUP);
   else if(PIN_Num == 7) _mcp -> pinMode(_mcp->eGPA7 , INPUT_PULLUP);
   else if(PIN_Num == 8) _mcp -> pinMode(_mcp->eGPB0 , INPUT_PULLUP);
   else if(PIN_Num == 9) _mcp -> pinMode(_mcp->eGPB1 , INPUT_PULLUP);
   else if(PIN_Num == 10) _mcp -> pinMode(_mcp->eGPB2 , INPUT_PULLUP);
   else if(PIN_Num == 11) _mcp -> pinMode(_mcp->eGPB3 , INPUT_PULLUP);
   else if(PIN_Num == 12) _mcp -> pinMode(_mcp->eGPB4 , INPUT_PULLUP);
   else if(PIN_Num == 13) _mcp -> pinMode(_mcp->eGPB5 , INPUT_PULLUP);
   else if(PIN_Num == 14) _mcp -> pinMode(_mcp->eGPB6 , INPUT_PULLUP);
   else if(PIN_Num == 15) _mcp -> pinMode(_mcp->eGPB7 , INPUT_PULLUP);

 
}
void MyInput::Set_toggle(bool value)
{
   this -> flag_toogle = value ;
   printf("set input flag %d  , pin : %d\n" , value , this -> PIN_NUM);
}
void MyInput::GetState(int Time)
{
  this -> OnDelayTime = Time;
  this -> OffDelayTime = Time;
  MyInput::GetState();
}
void MyInput::GetState()
{
  int PIN = this -> PIN_NUM;
  if(this -> flag_mcp)
  {
     if(PIN == 0) flag_state = _mcp -> digitalRead(_mcp->eGPA0);
     else if(PIN == 1) flag_state = _mcp -> digitalRead(_mcp->eGPA1);
     else if(PIN == 2) flag_state = _mcp -> digitalRead(_mcp->eGPA2);
     else if(PIN == 3) flag_state = _mcp -> digitalRead(_mcp->eGPA3);
     else if(PIN == 4) flag_state = _mcp -> digitalRead(_mcp->eGPA4);
     else if(PIN == 5) flag_state = _mcp -> digitalRead(_mcp->eGPA5);
     else if(PIN == 6) flag_state = _mcp -> digitalRead(_mcp->eGPA6);
     else if(PIN == 7) flag_state = _mcp -> digitalRead(_mcp->eGPA7);
     else if(PIN == 8) flag_state = _mcp -> digitalRead(_mcp->eGPB0);
     else if(PIN == 9) flag_state = _mcp -> digitalRead(_mcp->eGPB1);
     else if(PIN == 10) flag_state = _mcp -> digitalRead(_mcp->eGPB2);
     else if(PIN == 11) flag_state = _mcp -> digitalRead(_mcp->eGPB3);
     else if(PIN == 12) flag_state = _mcp -> digitalRead(_mcp->eGPB4);
     else if(PIN == 13) flag_state = _mcp -> digitalRead(_mcp->eGPB5);
     else if(PIN == 14) flag_state = _mcp -> digitalRead(_mcp->eGPB6);
     else if(PIN == 15) flag_state = _mcp -> digitalRead(_mcp->eGPB7);
  }
  else
  {
     this -> flag_state = digitalRead(this -> PIN_NUM);
  }
  
  if(this -> flag_toogle)
  {
    this -> flag_state = !(this -> flag_state);
  }
  if(this -> flag_state)
  {
     this -> cnt_off = 1;
     if(this -> cnt_on == 1)
     {
        myTimer.TickStop();
        myTimer.StartTickTime(this -> OnDelayTime);
        this -> cnt_on ++;
     }
     if(this -> cnt_on == 2)
     {
        if(myTimer.IsTimeOut())
        {
           this -> State = this -> flag_state;
           if(Input_ON != nullptr) Input_ON();
           //printf("Input ON PIN : %d , OnDelayTime : %d\n",PIN,OnDelayTime);
           this -> cnt_on ++;
        }
     }
  }
  else
  {
     this -> cnt_on = 1;
     if(this -> cnt_off == 1)
     {
        myTimer.TickStop();
        myTimer.StartTickTime(this -> OffDelayTime);
        this -> cnt_off ++;
     }
     if(this -> cnt_off == 2)
     {
        if(myTimer.IsTimeOut())
        {
           this -> State = this -> flag_state;
           if(Input_OFF != nullptr) Input_OFF();
           //printf("Input OFF PIN : %d , OnDelayTime : %d\n",PIN,OffDelayTime);
           this -> cnt_off ++;
        }
     }
  }
  
  
}
