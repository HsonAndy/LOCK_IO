#include "Arduino.h"
int cnt_UDP_Send = 65534;
MyTimer UDP_Send_Timer;
DynamicJsonDocument doc(1024);
String JsonOutput = "";

int RSSI;
void sub_UDP_Send()
{
  if(cnt_UDP_Send == 65534)
  {

      cnt_UDP_Send = 65535;
  }
  if(cnt_UDP_Send == 65535)
  {
      if(flag_boradInit)cnt_UDP_Send = 1;
  }
  if(cnt_UDP_Send == 1)
  {  
      if(wiFiConfig.uDP_SemdTime == 0)
      {
          cnt_UDP_Send = 1;
      }
      else
      {
          UDP_Send_Timer.TickStop();
          UDP_Send_Timer.StartTickTime(wiFiConfig.uDP_SemdTime);
          cnt_UDP_Send++;
      }
      
  }
  if(cnt_UDP_Send == 2)
  {      
      if(UDP_Send_Timer.IsTimeOut() || flag_JsonSend)
      {
         doc["IP"] = wiFiConfig.Get_IPAdress_Str();
         if((wiFiConfig.rFID_Enable >> 0) % 2 == 1)doc["CardID_01"] = CardID[0];
         if((wiFiConfig.rFID_Enable >> 1) % 2 == 1)doc["CardID_02"] = CardID[1];
         if((wiFiConfig.rFID_Enable >> 2) % 2 == 1)doc["CardID_03"] = CardID[2];
         if((wiFiConfig.rFID_Enable >> 3) % 2 == 1)doc["CardID_04"] = CardID[3];
         if((wiFiConfig.rFID_Enable >> 4) % 2 == 1)doc["CardID_05"] = CardID[4];
         doc["Port"] = wiFiConfig.Get_Localport();
         doc["RSSI"] = wiFiConfig.GetRSSI();        
         doc["Version"] = Version;
         doc["Input"] = Input;
         doc["Output"] = Output;
         doc["Input_dir"] = Input_dir;
         doc["Output_dir"] = Output_dir;
         doc["RFID_Enable"] = wiFiConfig.rFID_Enable;
         JsonOutput = "";
         serializeJson(doc, JsonOutput);
         Send_StringTo(JsonOutput, wiFiConfig.server_IPAdress, wiFiConfig.serverport);
         flag_JsonSend = false;
         cnt_UDP_Send++;
      }
  }
  if(cnt_UDP_Send == 3)
  {
     cnt_UDP_Send = 65535;
  }
}
