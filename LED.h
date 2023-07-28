#ifndef __LED_h
#define __LED_h
#include "Timer.h"
typedef void (*LEDHandle) (void);
class MyLED
{
   public:
   void Init(int PIN_Num);
   void Set_State(bool ON_OFF);
   void Blink(int Time);
   void Blink();
   int BlinkTime = -1;
   LEDHandle LED_ON = nullptr;
   LEDHandle LED_OFF = nullptr;
   private:
   int PIN_NUM = -1;
   int cnt = 1;
   MyTimer myTimer;
};

#endif
