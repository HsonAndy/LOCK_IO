#ifndef __Output_h
#define __Output_h
#include "Timer.h"
typedef void (*OutputHandle) (void);
#include "DFRobot_MCP23017.h"

#define cSetb0    0x0001
#define cSetb1    0x0002
#define cSetb2    0x0004
#define cSetb3    0x0008
#define cSetb4    0x0010
#define cSetb5    0x0020
#define cSetb6    0x0040
#define cSetb7    0x0080
#define cSetb8    0x0080
#define cSetb9    0x0080
#define cSetb10    0x0080
#define cSetb11    0x0080
#define cSetb12    0x0080
#define cSetb13    0x0080
#define cSetb14    0x0080
#define cSetb15    0x0080


class MyOutput
{
   public:
   DFRobot_MCP23017 *_mcp;
   bool flag_mcp = false;
   bool flag_toogle = false;
   bool Trigger = false;
   bool State = false;
   bool State_ON = false;
   bool State_OFF = false;
   bool BlinkEnable = false;
   void Init(int PIN_Num);
   void Init(int PIN_Num ,DFRobot_MCP23017& mcp);
   void Init(int PIN_Num , bool flag_toogle);
   void Set_toggle(bool value);
   void Set_State(bool ON_OFF);
   void Blink(int Time);
   void Blink();
   int OnDelayTime = -1;
   int OnDelayTime_buf = -2;
   OutputHandle Output_ON = nullptr;
   OutputHandle Output_OFF = nullptr;
   private:  
   bool state = false;
   bool GetLogic(bool value);
   int PIN_NUM = -1;
   int cnt = 254;
   MyTimer myTimer;
};


#endif
