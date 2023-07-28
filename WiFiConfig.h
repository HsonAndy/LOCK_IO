#ifndef __WiFiConfig_h
#define __WiFiConfig_h
#include "Arduino.h"
#include "UDP.h"
#include <WiFi.h>
#include <SoftwareSerial.h>

class WiFiConfig
{
   public:     
   SoftwareSerial *mySerial;
   bool flag_Init = false;
   bool IsConnected = false;
   IPAddress ipAdress;
   IPAddress server_IPAdress;
   String server_IPAdress_str = "";
   int localport;
   int serverport;
   int station;
   int uDP_SemdTime = 0;
   byte rFID_Enable = 0;
   int input_dir = 0;
   int output_dir = 0;
   int LED_OutputPIN[5];
   int LED_InputPIN[5];
   
   void WIFI_Connenct();
   void WIFI_Disconnenct();
   void Init(String Version);
   void Set_IPAdress(byte IP0 ,byte IP1 ,byte IP2 ,byte IP3);
   void Set_Subnet(byte IP0 ,byte IP1 ,byte IP2 ,byte IP3);  
   void Set_Gateway(byte IP0 ,byte IP1 ,byte IP2 ,byte IP3);
   void Set_DNS(byte IP0 ,byte IP1 ,byte IP2 ,byte IP3);  
   void Set_Server_IPAdress(byte IP0 ,byte IP1 ,byte IP2 ,byte IP3);  
   void Set_Localport(int port);
   void Set_Serverport(int port);
   void Set_SSID(String ssid);
   void Set_Password(String password);
   void Set_Station(int station);
   void Set_PC_Restart(bool state);
   void Set_IsUpdate(bool state);
   void Set_UDP_SemdTime(int ms);
   void Set_Input_dir(int value);
   void Set_Output_dir(int value);
   void Set_RFID_Enable(byte value);
   void Set_LED_OutputPIN(int index , int pinNum);
   void Set_LED_InputPIN(int index , int pinNum);
   void HandleClient();
   void httpInit();
  
   int GetRSSI();
   byte* Get_IPAdress();
   byte* Get_Subnet();
   byte* Get_Gateway();
   byte* Get_DNS();
   byte* Get_Server_IPAdress();
   int Get_Localport();  
   int Get_Serverport();
   int Get_Station();
   byte Get_PC_Restart();
   byte Get_IsUpdate();
   int Get_UDP_SemdTime();
   byte Get_RFID_Enable();
   int Get_Input_dir();
   int Get_Output_dir();
   int Get_LED_OutputPIN(int index);
   int Get_LED_InputPIN(int index);
   
   String Get_IPAdress_Str();
   IPAddress Get_IPAdressClass();
   String Get_Subnet_Str();
   String Get_Gateway_Str();
   String Get_DNS_Str();
   String Get_Server_IPAdress_Str();
   IPAddress Get_Server_IPAdressClass();
   String Get_Localport_Str();
   String Get_Serverport_Str();
   String Get_SSID_Str();
   String Get_Password_Str();
   String Get_Station_Str();
   String Get_UDP_SemdTime_Str();
   String Get_RFID_Enable_Str();
   
   private:
   byte iP_Adress[4];
   byte subnet[4];
   byte gateway[4];
   byte dns[4];
   byte server_iP_Adress[4];
   
   const int iP_Adress_ADDR[4] = {0 ,1 ,2 ,3};
   const int subnet_ADDR[4] = {4 ,5 ,6 ,7};
   const int gateway_ADDR[4] = {8 ,9 ,10 ,11};
   const int dns_ADDR[4] = {12 ,13 ,14 ,15};
   const int localport_ADDR[2] = {16 ,18};
   const int server_iP_Adress_ADDR[4] = {19 ,20 ,21 ,22};
   const int serverport_ADDR[2] = {23 ,24};  
   const int station_ADDR[2] = {25 ,26};  
   const int pc_restart_ADDR = 27;
   const int IsUpdate_ADDR = 28;
   const int UDP_SemdTime_ADDR[2] = {29 ,30}; 
   const int RFID_Enable_ADDR = 31;  
   const int Input_dir_ADDR[2] = {32 ,33}; 
   const int Output_dir_ADDR[2] = {34 ,35};   
   const int SSID_ADDR = 40;
   const int SSID_SIZE = 20;
   const int Password_ADDR = 60;
   const int Password_SIZE = 20; 
   const int LED_OutputPIN_ADDR[5] = {100 , 101 , 102 , 103 , 104};
   const int LED_InputPIN_ADDR[5] = {110 , 111 , 112 , 113 , 114};
};

#endif
