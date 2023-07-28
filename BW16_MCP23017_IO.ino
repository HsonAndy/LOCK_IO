#include "WS2812B.h"
#include <WiFi.h>
#include <WiFiUdp.h> 
#include <FreeRTOS.h>
#include <task.h>
#include <FlashMemory.h>
#include "Timer.h"
#include "LED.h"
#include "Output.h"
#include "Input.h" 
#include "WiFiConfig.h"
#include "MyJPEGDecoder.h"
#include "MyWS2812.h"
#include <ArduinoJson.h>
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include "DFRobot_MCP23017.h"

#define INPUT_PIN01 0
#define OUTPUT_PIN01 8
#define NUM_WS2812B_CRGB  450
#define NUM_OF_LEDS NUM_WS2812B_CRGB
#define SYSTEM_LED_PIN PA30
#define PIN_485_Tx_Eanble PB3

DFRobot_MCP23017 mcp(Wire, /*addr =*/0x20);//constructor, change the Level of A2, A1, A0 via DIP switch to revise the I2C address within 0x20~0x27.
MyInput MyInput_01;
MyOutput MyOutput_01;

bool flag_output_Init = false;
bool flag_udp_232back = false;
bool flag_JsonSend = false;
bool flag_writeMode = false;

bool flag_CardID_IsChanged = false;
String CardID[5];
String CardID_buf[5];

WiFiConfig wiFiConfig;
int UDP_SemdTime = 0;
int Localport = 0;
IPAddress ServerIp;
int Serverport;
String GetwayStr;

bool flag_WS2812B_Refresh = false;
MyWS2812 myWS2812;

byte* framebuffer;
byte OUTPIN = 1;
MyTimer MyTimer_BoardInit;
bool flag_boradInit = false;
bool flag_ioInit = false;
MyLED MyLED_IS_Connented;

bool flag_MyInput_01_on = false;
TaskHandle_t Core0Task1Handle;
TaskHandle_t Core0Task2Handle;
TaskHandle_t Core0Task3Handle;
TaskHandle_t Core0Task4Handle;

bool flag_PIN01_OUT = false;
bool flag_PIN02_OUT = false;
bool flag_PIN03_OUT = false;
bool flag_PIN04_OUT = false;
bool flag_PIN05_OUT = false;
bool flag_PIN06_OUT = false;
bool flag_PIN07_OUT = false;
bool flag_PIN08_OUT = false;
bool flag_PIN09_OUT = false;
bool flag_PIN10_OUT = false;


MyTimer MyTimer_PIN01_OUT;
MyTimer MyTimer_PIN02_OUT;
MyTimer MyTimer_PIN03_OUT;
MyTimer MyTimer_PIN04_OUT;
MyTimer MyTimer_PIN05_OUT;
MyTimer MyTimer_PIN06_OUT;
MyTimer MyTimer_PIN07_OUT;
MyTimer MyTimer_PIN08_OUT;
MyTimer MyTimer_PIN09_OUT;
MyTimer MyTimer_PIN10_OUT;

SoftwareSerial mySerial(PA8, PA7); // RX, TX
SoftwareSerial mySerial_485(PB2, PB1); // RX, TX

String Version = "Ver 1.0.3";

void setup() 
{
    
    mySerial.begin(115200);   
    mySerial.println(Version);
    MyTimer_BoardInit.StartTickTime(1000);   
    while(mcp.begin() != 0)
    {
      Serial.println("Initialization of the chip failed, please confirm that the chip connection is correct!");
      delay(1000);
    };
    
  mcp.pinMode(/*pin = */mcp.eGPA0, /*mode = */INPUT_PULLUP);
  mcp.pinMode(/*pin = */mcp.eGPA1, /*mode = */INPUT_PULLUP);
  mcp.pinMode(/*pin = */mcp.eGPA2, /*mode = */INPUT_PULLUP);
  mcp.pinMode(/*pin = */mcp.eGPA3, /*mode = */INPUT_PULLUP);
  mcp.pinMode(/*pin = */mcp.eGPA4, /*mode = */INPUT_PULLUP);
  mcp.pinMode(/*pin = */mcp.eGPA5, /*mode = */INPUT_PULLUP);
  mcp.pinMode(/*pin = */mcp.eGPA6, /*mode = */INPUT_PULLUP);
  mcp.pinMode(/*pin = */mcp.eGPA7, /*mode = */INPUT_PULLUP);

  mcp.pinMode(/*pin = */mcp.eGPB0, /*mode = */OUTPUT);
  mcp.pinMode(/*pin = */mcp.eGPB1, /*mode = */OUTPUT);
  mcp.pinMode(/*pin = */mcp.eGPB2, /*mode = */OUTPUT);
  mcp.pinMode(/*pin = */mcp.eGPB3, /*mode = */OUTPUT);
  mcp.pinMode(/*pin = */mcp.eGPB4, /*mode = */OUTPUT);
  mcp.pinMode(/*pin = */mcp.eGPB5, /*mode = */OUTPUT);
  mcp.pinMode(/*pin = */mcp.eGPB6, /*mode = */OUTPUT);
  mcp.pinMode(/*pin = */mcp.eGPB7, /*mode = */OUTPUT);

  pinMode(PA15, OUTPUT);
  pinMode(PA27, OUTPUT);
  pinMode(PA13, INPUT_PULLUP);
  pinMode(PA14, INPUT_PULLUP);
}
void loop() 
{
      if(MyTimer_BoardInit.IsTimeOut())
   {  
    uint8_t PIN01 = mcp.digitalRead(/*pin = */mcp.eGPA0);
    uint8_t PIN02 = mcp.digitalRead(/*pin = */mcp.eGPA1);
    uint8_t PIN03 = mcp.digitalRead(/*pin = */mcp.eGPA2);
    uint8_t PIN04 = mcp.digitalRead(/*pin = */mcp.eGPA3);
    uint8_t PIN05 = mcp.digitalRead(/*pin = */mcp.eGPA4);
    uint8_t PIN06 = mcp.digitalRead(/*pin = */mcp.eGPA5);
    uint8_t PIN07 = mcp.digitalRead(/*pin = */mcp.eGPA6);
    uint8_t PIN08 = mcp.digitalRead(/*pin = */mcp.eGPA7);

    uint8_t PIN09 = digitalRead(PA13); 
    uint8_t PIN10 = digitalRead(PA14); 


  if(!PIN01)
  {
    flag_PIN10_OUT = true;
  }
  if(!PIN02)
  {
    flag_PIN09_OUT = true;
  }
  if(!PIN03)
  {
    flag_PIN08_OUT = true;
  }
  if(!PIN04)
  {
    flag_PIN07_OUT = true;
  }
  if(!PIN05)
  {
    flag_PIN06_OUT = true;
  }
  if(!PIN06)
  {
    flag_PIN05_OUT = true;
  }
  if(!PIN07)
  {
    flag_PIN04_OUT = true;
  }
  if(!PIN08)
  {
    flag_PIN03_OUT = true;
  }
  if(!PIN09)
  {
    flag_PIN02_OUT = true;
  }
  if(!PIN10)
  {
    flag_PIN01_OUT = true;
  }


  if(flag_PIN01_OUT)
  {
    mcp.digitalWrite(mcp.eGPB0 , true);
    MyTimer_PIN01_OUT.StartTickTime(3000);
    if(MyTimer_PIN01_OUT.IsTimeOut())
    {
      mcp.digitalWrite(mcp.eGPB0 , false);
      flag_PIN01_OUT = false;
    }


  }
  if(flag_PIN02_OUT)
  {
    mcp.digitalWrite(mcp.eGPB1 , true);
    MyTimer_PIN02_OUT.StartTickTime(3000);
    if(MyTimer_PIN02_OUT.IsTimeOut())
    {
      mcp.digitalWrite(mcp.eGPB1 , false);
      flag_PIN02_OUT = false;
    }

  }
  if(flag_PIN03_OUT)
  {
    mcp.digitalWrite(mcp.eGPB2 , true);
    MyTimer_PIN03_OUT.StartTickTime(3000);
    if(MyTimer_PIN03_OUT.IsTimeOut())
    {
      mcp.digitalWrite(mcp.eGPB2 , false);
      flag_PIN03_OUT = false;
    }

  }
  if(flag_PIN04_OUT)
  {
    mcp.digitalWrite(mcp.eGPB3 , true);
    MyTimer_PIN04_OUT.StartTickTime(3000);
    if(MyTimer_PIN04_OUT.IsTimeOut())
    {
      mcp.digitalWrite(mcp.eGPB3 , false);
      flag_PIN04_OUT = false;
    }

  }
  if(flag_PIN05_OUT)
  {
    mcp.digitalWrite(mcp.eGPB4 , true);
    MyTimer_PIN05_OUT.StartTickTime(3000);
    if(MyTimer_PIN05_OUT.IsTimeOut())
    {
      mcp.digitalWrite(mcp.eGPB4 , false);
      flag_PIN05_OUT = false;
    }

  }
  if(flag_PIN06_OUT)
  {

     mcp.digitalWrite(mcp.eGPB5 , true);
    MyTimer_PIN06_OUT.StartTickTime(3000);
    if(MyTimer_PIN06_OUT.IsTimeOut())
    {
      mcp.digitalWrite(mcp.eGPB5 , false);
      flag_PIN06_OUT = false;
    }

  }
  if(flag_PIN07_OUT)
  {

     mcp.digitalWrite(mcp.eGPB6 , true);
    MyTimer_PIN07_OUT.StartTickTime(3000);
    if(MyTimer_PIN07_OUT.IsTimeOut())
    {
      mcp.digitalWrite(mcp.eGPB6 , false);
      flag_PIN07_OUT = false;
    }

  }
  if(flag_PIN08_OUT)
  {
     mcp.digitalWrite(mcp.eGPB7 , true);
    MyTimer_PIN08_OUT.StartTickTime(3000);
    if(MyTimer_PIN08_OUT.IsTimeOut())
    {
      mcp.digitalWrite(mcp.eGPB7 , false);
      flag_PIN08_OUT = false;
    }

  }
  if(flag_PIN09_OUT)
  {

     digitalWrite(PA15,true);
    MyTimer_PIN09_OUT.StartTickTime(3000);
    if(MyTimer_PIN09_OUT.IsTimeOut())
    {
      digitalWrite(PA15,false);
      flag_PIN09_OUT = false;
    }

  }
  if(flag_PIN10_OUT)
  {
    digitalWrite(PA27,true);
    MyTimer_PIN10_OUT.StartTickTime(3000);
    if(MyTimer_PIN10_OUT.IsTimeOut())
    {
      digitalWrite(PA27,false);
      flag_PIN10_OUT = false;
    }
  }

   }
}


