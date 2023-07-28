#include "WiFiConfig.h"
#include <EEPROM.h>
#include "Arduino.h"
#include "UDP.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <Wire.h>

void WiFiConfig::Init(String Version)
{
    EEPROM.begin(1024);
    String _SSID = "SSID : " + this -> Get_SSID_Str();
    String _Password = "Password : " + this -> Get_Password_Str();
    String ipAdress = "IPAdress : " + this -> Get_IPAdress_Str();
    server_IPAdress_str = this -> Get_Server_IPAdress_Str();
    String server_ipAdress = "Server IPAdress : " + server_IPAdress_str;
  
    String subnet = "Subnet : " + this -> Get_Subnet_Str();
    String gateway = "Gateway : " + this -> Get_Gateway_Str();
    String dns = "Dns : " + this -> Get_DNS_Str();
    String _Localport = "Localport : " + this -> Get_Localport_Str();
    String _Serverport = "Serverport : " + this -> Get_Serverport_Str();
    byte mac[6];
    WiFi.macAddress(mac);
    String HEX_0 =  String(mac[0], HEX); 
    String HEX_1 =  String(mac[1], HEX); 
    String HEX_2 =  String(mac[2], HEX); 
    String HEX_3 =  String(mac[3], HEX); 
    String HEX_4 =  String(mac[4], HEX); 
    String HEX_5 =  String(mac[5], HEX);   
    String MacAdress = "MacAdress :" + HEX_0 + ":"+ HEX_1 + ":"+ HEX_2 + ":"+ HEX_3 + ":"+ HEX_4 + ":"+ HEX_5;   
    String PC_Restart = "PC Restart :" +   String(this -> Get_PC_Restart());
    String IsUpdate = "PC IsUpdate :" +   String(this -> Get_IsUpdate());
    
    mySerial -> println(ipAdress);
    mySerial -> println(_Localport);
    mySerial -> println(subnet);
    mySerial -> println(gateway);
    mySerial -> println(dns);   
    mySerial -> println(server_ipAdress); 
    mySerial -> println(_Serverport);
    mySerial -> println(_SSID);
    mySerial -> println(_Password);   
    mySerial -> println(MacAdress);  
    mySerial -> println(PC_Restart);   
    mySerial -> println(IsUpdate); 

    //this -> WIFI_Connenct();

   
}

void WiFiConfig::WIFI_Disconnenct()
{
    WiFi.disconnect();
}
void WiFiConfig::WIFI_Connenct()
{
    WiFi.disablePowerSave();
    WiFi.disconnect();
   
    byte* ipAdress_ptr = this -> Get_IPAdress();
    byte* gateway_ptr = this -> Get_Gateway();
    byte* subnet_ptr = this -> Get_Subnet();
    byte* dns_ptr = this -> Get_DNS();
    IPAddress ipAdress(*(ipAdress_ptr + 0 ),*(ipAdress_ptr + 1 ),*(ipAdress_ptr + 2 ),*(ipAdress_ptr + 3 ));
    IPAddress gateway(*(gateway_ptr + 0 ),*(gateway_ptr + 1 ),*(gateway_ptr + 2 ),*(gateway_ptr + 3 ));
    IPAddress subnet(*(subnet_ptr + 0 ),*(subnet_ptr + 1 ),*(subnet_ptr + 2 ),*(subnet_ptr + 3 ));
    IPAddress dns1(168,95,1,1);
    IPAddress dns2(*(dns_ptr + 0 ),*(dns_ptr + 1 ),*(dns_ptr + 2 ),*(dns_ptr + 3 ));
    WiFi.config(ipAdress ,dns2 ,gateway ,subnet );
   
  
    String _SSID =  this -> Get_SSID_Str();
    char _ssid[sizeof(_SSID)];
    _SSID.toCharArray(_ssid , sizeof(_SSID) );
    
    String _Password =  this -> Get_Password_Str();
    char _password[sizeof(_Password)];
    _Password.toCharArray(_password , sizeof(_Password));
    WiFi.begin(_ssid, _password);
    if(!this -> flag_Init)printf("connecting.");
    int retry = 0;
    while (WiFi.status() != WL_CONNECTED) //等待网络连接成功
    {
      if( retry >= 5)
      {
         break;
      }
      delay(500);
      if(!this -> flag_Init)printf(".");
      retry++;
    }
     if(!this -> flag_Init)printf("\n\r");
    this -> IsConnected = (WiFi.status() == WL_CONNECTED);
    if( this -> IsConnected)
    {
      if(!this -> flag_Init)printf("WiFi connected!\n");
      if(!this -> flag_Init)printf("IP address: %d\n",WiFi.localIP());
    }
    else
    {
      if(!this -> flag_Init)printf("WiFi connect failed!\n");      
    }
    this -> flag_Init = true;
    
}

void WiFiConfig::Set_IPAdress(byte IP0 ,byte IP1 ,byte IP2 ,byte IP3)
{
    EEPROM.write(this -> iP_Adress_ADDR[0] , IP0);
    EEPROM.write(this -> iP_Adress_ADDR[1] , IP1);
    EEPROM.write(this -> iP_Adress_ADDR[2] , IP2);
    EEPROM.write(this -> iP_Adress_ADDR[3] , IP3);
    EEPROM.commit();
    IPAddress ipAdress(IP0,IP1,IP2,IP3);
    this -> ipAdress = ipAdress;
}
void WiFiConfig::Set_Subnet(byte IP0 ,byte IP1 ,byte IP2 ,byte IP3)
{
    EEPROM.write(this -> subnet_ADDR[0] , IP0);
    EEPROM.write(this -> subnet_ADDR[1] , IP1);
    EEPROM.write(this -> subnet_ADDR[2] , IP2);
    EEPROM.write(this -> subnet_ADDR[3] , IP3);
    EEPROM.commit();
}
void WiFiConfig::Set_Gateway(byte IP0 ,byte IP1 ,byte IP2 ,byte IP3)
{
    EEPROM.write(this -> gateway_ADDR[0] , IP0);
    EEPROM.write(this -> gateway_ADDR[1] , IP1);
    EEPROM.write(this -> gateway_ADDR[2] , IP2);
    EEPROM.write(this -> gateway_ADDR[3] , IP3);
    EEPROM.commit();
}
void WiFiConfig::Set_DNS(byte IP0 ,byte IP1 ,byte IP2 ,byte IP3)
{
    EEPROM.write(this -> dns_ADDR[0] , IP0);
    EEPROM.write(this -> dns_ADDR[1] , IP1);
    EEPROM.write(this -> dns_ADDR[2] , IP2);
    EEPROM.write(this -> dns_ADDR[3] , IP3);
    EEPROM.commit();
}
void WiFiConfig::Set_Server_IPAdress(byte IP0 ,byte IP1 ,byte IP2 ,byte IP3)
{
    EEPROM.write(this -> server_iP_Adress_ADDR[0] , IP0);
    EEPROM.write(this -> server_iP_Adress_ADDR[1] , IP1);
    EEPROM.write(this -> server_iP_Adress_ADDR[2] , IP2);
    EEPROM.write(this -> server_iP_Adress_ADDR[3] , IP3);
    EEPROM.commit();
    IPAddress ipAdress(IP0,IP1,IP2,IP3);
    this -> server_IPAdress = ipAdress;
}
void WiFiConfig::Set_Localport(int port)
{
    byte L = port;
    byte H = (port >> 8);
    EEPROM.write(this -> localport_ADDR[0] , L);
    EEPROM.write(this -> localport_ADDR[1] , H);
    EEPROM.commit();
    this -> localport = port;
}
void WiFiConfig::Set_Serverport(int port)
{
    byte L = port;
    byte H = (port >> 8);
    EEPROM.write(this -> serverport_ADDR[0] , L);
    EEPROM.write(this -> serverport_ADDR[1] , H);
    EEPROM.commit();
    this -> serverport = port;
}
void WiFiConfig::Set_SSID(String ssid)
{  
  byte buf[20];
  ssid.getBytes(buf , 20);
  for(int i = 0 ; i < this -> SSID_SIZE ; i++)
  {
     EEPROM.write((this -> SSID_ADDR) + i , buf[i]);
  }
  EEPROM.commit();
}
void WiFiConfig::Set_Password(String password)
{  
  byte buf[20];
  password.getBytes(buf , 20);
  for(int i = 0 ; i < this -> Password_SIZE ; i++)
  {
     EEPROM.write((this -> Password_ADDR) + i , buf[i]);
  }
  EEPROM.commit();
}
void WiFiConfig::Set_Station(int station)
{
    byte L = station;
    byte H = (station >> 8);
    EEPROM.write(this -> station_ADDR[0] , L);
    EEPROM.write(this -> station_ADDR[1] , H);
    EEPROM.commit();
}
void WiFiConfig::Set_PC_Restart(bool state)
{
    if(state)
    {
      EEPROM.write(this -> pc_restart_ADDR , 1);
    }
    else
    {
      EEPROM.write(this -> pc_restart_ADDR , 0);
    }    
    EEPROM.commit();
}
void WiFiConfig::Set_IsUpdate(bool state)
{
    if(state)
    {
      EEPROM.write(this -> IsUpdate_ADDR , 1);
    }
    else
    {
      EEPROM.write(this -> IsUpdate_ADDR , 0);
    }    
    EEPROM.commit();
}
void WiFiConfig::Set_UDP_SemdTime(int ms)
{
    byte L = ms;
    byte H = (ms >> 8);
    this -> uDP_SemdTime = ms;
    EEPROM.write(this -> UDP_SemdTime_ADDR[0] , L);
    EEPROM.write(this -> UDP_SemdTime_ADDR[1] , H);
    EEPROM.commit();
}
void WiFiConfig::Set_RFID_Enable(byte value)
{
    EEPROM.write(this -> RFID_Enable_ADDR , value);
    EEPROM.commit();
    this -> rFID_Enable = value;
}
void WiFiConfig::Set_Input_dir(int value)
{
    byte value_L = value;
    byte value_H = (value >> 8);
    EEPROM.write(this -> Input_dir_ADDR[0] , value_L);
    EEPROM.write(this -> Input_dir_ADDR[1] , value_H);
    EEPROM.commit();
    this -> input_dir = value;
}
void WiFiConfig::Set_Output_dir(int value)
{
    byte value_L = value;
    byte value_H = (value >> 8);
    EEPROM.write(this -> Output_dir_ADDR[0] , value_L);
    EEPROM.write(this -> Output_dir_ADDR[1] , value_H);
    EEPROM.commit();
    this -> output_dir = value;
}
int WiFiConfig::Get_LED_OutputPIN(int index)
{
    byte value = EEPROM.read(this -> LED_OutputPIN_ADDR[index]);
    this -> LED_OutputPIN[index] = value;
    return this -> LED_OutputPIN[index];
}
int WiFiConfig::Get_LED_InputPIN(int index)
{
    byte value = EEPROM.read(this -> LED_InputPIN_ADDR[index]);
    this -> LED_InputPIN[index] = value;
    return this -> LED_InputPIN[index];
}
void WiFiConfig::Set_LED_OutputPIN(int index , int pinNum)
{
    EEPROM.write(this -> LED_OutputPIN_ADDR[index] , pinNum);
    EEPROM.commit();
    this -> LED_OutputPIN[index] = pinNum;
}
void WiFiConfig::Set_LED_InputPIN(int index , int pinNum)
{
    EEPROM.write(this -> LED_InputPIN_ADDR[index] , pinNum);
    EEPROM.commit();
    this -> LED_InputPIN[index] = pinNum;
}
////////////////////////////////////////////////////////////////////////////////////////////////

int WiFiConfig::GetRSSI()
{
    return WiFi.RSSI();
}
byte* WiFiConfig::Get_IPAdress()
{
    this -> iP_Adress[0] = EEPROM.read(this -> iP_Adress_ADDR[0]);
    this -> iP_Adress[1] = EEPROM.read(this -> iP_Adress_ADDR[1]);
    this -> iP_Adress[2] = EEPROM.read(this -> iP_Adress_ADDR[2]);
    this -> iP_Adress[3] = EEPROM.read(this -> iP_Adress_ADDR[3]);
    return this -> iP_Adress;
}
byte* WiFiConfig::Get_Subnet()
{
    this -> subnet[0] = EEPROM.read(this -> subnet_ADDR[0]);
    this -> subnet[1] = EEPROM.read(this -> subnet_ADDR[1]);
    this -> subnet[2] = EEPROM.read(this -> subnet_ADDR[2]);
    this -> subnet[3] = EEPROM.read(this -> subnet_ADDR[3]);
    return this -> subnet;
}
byte* WiFiConfig::Get_Gateway()
{
    this -> gateway[0] = EEPROM.read(this -> gateway_ADDR[0]);
    this -> gateway[1] = EEPROM.read(this -> gateway_ADDR[1]);
    this -> gateway[2] = EEPROM.read(this -> gateway_ADDR[2]);
    this -> gateway[3] = EEPROM.read(this -> gateway_ADDR[3]);
    return this -> gateway;
}
byte* WiFiConfig::Get_DNS()
{
    this -> dns[0] = EEPROM.read(this -> dns_ADDR[0]);
    this -> dns[1] = EEPROM.read(this -> dns_ADDR[1]);
    this -> dns[2] = EEPROM.read(this -> dns_ADDR[2]);
    this -> dns[3] = EEPROM.read(this -> dns_ADDR[3]);
    return this -> dns;
}
byte* WiFiConfig::Get_Server_IPAdress()
{
    this -> server_iP_Adress[0] = EEPROM.read(this -> server_iP_Adress_ADDR[0]);
    this -> server_iP_Adress[1] = EEPROM.read(this -> server_iP_Adress_ADDR[1]);
    this -> server_iP_Adress[2] = EEPROM.read(this -> server_iP_Adress_ADDR[2]);
    this -> server_iP_Adress[3] = EEPROM.read(this -> server_iP_Adress_ADDR[3]);
    return this -> server_iP_Adress;
}
int WiFiConfig::Get_Localport()
{
    byte L = EEPROM.read(this -> localport_ADDR[0]);
    byte H = EEPROM.read(this -> localport_ADDR[1]);
    this -> localport = ( L | (H << 8));
    return this -> localport;
}
int WiFiConfig::Get_Serverport()
{
    byte L = EEPROM.read(this -> serverport_ADDR[0]);
    byte H = EEPROM.read(this -> serverport_ADDR[1]);
    this -> serverport = ( L | (H << 8));
    return this -> serverport;
}
int WiFiConfig::Get_Station()
{
    byte L = EEPROM.read(this -> station_ADDR[0]);
    byte H = EEPROM.read(this -> station_ADDR[1]);
    this -> station = ( L | (H << 8));
    return this -> station;
}
byte WiFiConfig::Get_PC_Restart()
{
    byte flag = EEPROM.read(this -> pc_restart_ADDR);    
    return flag;
}
byte WiFiConfig::Get_IsUpdate()
{
    byte flag = EEPROM.read(this -> IsUpdate_ADDR);    
    return flag;
}
int WiFiConfig::Get_UDP_SemdTime()
{
    byte L = EEPROM.read(this -> UDP_SemdTime_ADDR[0]);
    byte H = EEPROM.read(this -> UDP_SemdTime_ADDR[1]);
    this -> uDP_SemdTime = ( L | (H << 8));
    return this -> uDP_SemdTime;
}
byte WiFiConfig::Get_RFID_Enable()
{
    byte value = EEPROM.read(this -> RFID_Enable_ADDR);
    this -> rFID_Enable = value;
    return this -> rFID_Enable;
}
int WiFiConfig::Get_Input_dir()
{
    byte L = EEPROM.read(this -> Input_dir_ADDR[0]);
    byte H = EEPROM.read(this -> Input_dir_ADDR[1]);
    this -> input_dir = ( L | (H << 8));
    return this -> input_dir;
}
int WiFiConfig::Get_Output_dir()
{
    byte L = EEPROM.read(this -> Output_dir_ADDR[0]);
    byte H = EEPROM.read(this -> Output_dir_ADDR[1]);
    this -> output_dir = ( L | (H << 8));
    return this -> output_dir;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////

String WiFiConfig::Get_IPAdress_Str()
{
    byte* ip; 
    ip = this -> Get_IPAdress();
    IPAddress ipAdress(*(ip + 0 ),*(ip + 1 ),*(ip + 2 ),*(ip + 3 ));
    this -> ipAdress = ipAdress;
    return String(*(ip + 0)) + '.' + String(*(ip + 1)) + '.' + String(*(ip + 2)) + '.' + String(*(ip + 3));
}
IPAddress WiFiConfig::Get_IPAdressClass()
{
    byte* ip; 
    ip = this -> Get_IPAdress();
    IPAddress ipAdress(*(ip + 0 ),*(ip + 1 ),*(ip + 2 ),*(ip + 3 ));
    this -> ipAdress = ipAdress;
    return ipAdress;
}
String WiFiConfig::Get_Subnet_Str()
{
    byte* ip; 
    ip = this -> Get_Subnet();
    return String(*(ip + 0)) + '.' + String(*(ip + 1)) + '.' + String(*(ip + 2)) + '.' + String(*(ip + 3));
}
String WiFiConfig::Get_Gateway_Str()
{
    byte* ip; 
    ip = this -> Get_Gateway();
    return String(*(ip + 0)) + '.' + String(*(ip + 1)) + '.' + String(*(ip + 2)) + '.' + String(*(ip + 3));
}
String WiFiConfig::Get_DNS_Str()
{
    byte* ip; 
    ip = this -> Get_DNS();
    return String(*(ip + 0)) + '.' + String(*(ip + 1)) + '.' + String(*(ip + 2)) + '.' + String(*(ip + 3));
}
String WiFiConfig::Get_Server_IPAdress_Str()
{
    byte* ip; 
    ip = this -> Get_Server_IPAdress();
    IPAddress ipAdress(*(ip + 0 ),*(ip + 1 ),*(ip + 2 ),*(ip + 3 ));
    this -> server_IPAdress = ipAdress;
    return String(*(ip + 0)) + '.' + String(*(ip + 1)) + '.' + String(*(ip + 2)) + '.' + String(*(ip + 3));
}
IPAddress WiFiConfig::Get_Server_IPAdressClass()
{
    byte* ip; 
    ip = this -> Get_Server_IPAdress();
    IPAddress ipAdress(*(ip + 0 ),*(ip + 1 ),*(ip + 2 ),*(ip + 3 ));
    this -> server_IPAdress = ipAdress;
    return ipAdress;
}
String WiFiConfig::Get_Localport_Str()
{
    return String(this -> Get_Localport());
}
String WiFiConfig::Get_Serverport_Str()
{
    return String(this -> Get_Serverport());
}
String WiFiConfig::Get_SSID_Str()
{
    String str_SSID = "";
    for(int i = 0 ; i < this -> SSID_SIZE ; i++)
    {
       char temp = EEPROM.read((this -> SSID_ADDR) + i);
       if(temp != 0)
       {
         str_SSID+= temp;
       }
       else
       {
         break;
       }
    }
    return str_SSID;
}
String WiFiConfig::Get_Password_Str()
{
    String str_Password = "";
    for(int i = 0 ; i < this -> Password_SIZE ; i++)
    {
       char temp = EEPROM.read((this -> Password_ADDR) + i);
       if(temp != 0)
       {
         str_Password+= temp;
       }
       else
       {
         break;
       }
    }
    return str_Password;
}
String WiFiConfig::Get_Station_Str()
{
    return String(this -> Get_Station());
}
String WiFiConfig::Get_UDP_SemdTime_Str()
{
    return String(this -> Get_UDP_SemdTime());
}
String WiFiConfig::Get_RFID_Enable_Str()
{
    return String(this -> Get_RFID_Enable());
}
