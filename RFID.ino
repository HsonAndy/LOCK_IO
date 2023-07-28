MyTimer MyTimer_RFID;
#define RFID_RX_SIZE 512
static byte RFID_RX[RFID_RX_SIZE];

int RFID_len;
byte rFID_Enable = 0;
byte rFID_Enable_buf = -1;
bool RFID_Init = true;
String CardID_temp = "";
int RFID_Error = 0;
void sub_RFID_program()
{
    rFID_Enable = Get_RFID_Enable();
    if(rFID_Enable_buf != rFID_Enable)
    {       
       if(flag_udp_232back)printf("RFID_Enable : %d\n" , rFID_Enable);
       rFID_Enable_buf = rFID_Enable;
       flag_JsonSend = true;
    }
    if(RFID_Init) 
    {
      wiFiConfig.Get_RFID_Enable();
      
    }
    for(int i = 0 ; i < 5 ; i++)
    {
      if(RFID_Init)
      {
        if(((wiFiConfig.rFID_Enable >> i) % 2) == 1)Set_Beep(i + 1);     
        CardID[i] = "00000000000000";
        CardID_buf[i] = "00000000000000";     
      }
      else
      {
        if(((wiFiConfig.rFID_Enable >> i) % 2) == 1)
        {
            CardID_temp = Get_7CardID(i + 1);
            CardID[i] = CardID_temp;
            
        }
        else
        {
           CardID[i] = "00000000000000";
           CardID_buf[i] = "00000000000000";     
        } 
      }
      if(RFID_Error >= 20)
      {
//         ESP.restart();       
      }
    }
    RFID_Init = false;
    
}
void Set_RFID_Enable(byte index , bool value)
{
    byte temp = wiFiConfig.Get_RFID_Enable();
    
    if(value)
    {
      temp = temp |(1 << index);
    }    
    else
    {
      temp = temp & ~(1 << index);
    }
    wiFiConfig.Set_RFID_Enable(temp);
}
byte Get_RFID_Enable()
{
    return wiFiConfig.rFID_Enable;
}
String Get_7CardID(byte station)
{
   int retry = 0;
   byte tx[8];
   tx[0] = station;
   tx[1] = 0x03;
   tx[2] = 0x00;
   tx[3] = 0x00;
   tx[4] = 0x00;
   tx[5] = 0x04;
   uint16_t CRC = Get_CRC16(tx , 6);
   tx[6] = CRC;
   tx[7] = (CRC >> 8);
   byte flag_rx_ok = 0;
   byte read_temp;
   bool flag_rx_Start = false;
   while(true)
   {
     if(retry >= 3) 
     {
        RFID_Error++;
        break;
     }
     flag_rx_ok = 0;
     flag_rx_Start = false;
     Set_RS485_Tx_Enable();
     mySerial_485.write(tx , 8);
     mySerial_485.flush();     
     Set_RS485_Rx_Enable();
     RFID_len = 0;
     for (int i = 0 ; i < RFID_RX_SIZE ; i++)
     {
         RFID_RX[i] = 0;         
     }
     MyTimer_RFID.TickStop();
     MyTimer_RFID.StartTickTime(100);
     while(true)
     {
        if (mySerial_485.available())
        {
          read_temp = mySerial_485.read();
          if(read_temp == station)
          {
             flag_rx_Start = true;
          }
          if(flag_rx_Start)
          {
            RFID_RX[RFID_len] = read_temp;
//            if(flag_udp_232back)printf("value: %d , RFID_len : %d \n" ,read_temp,RFID_len);
            RFID_len++;
            MyTimer_RFID.TickStop();
            MyTimer_RFID.StartTickTime(100);
            
          }
          
         
        }
        if(RFID_len >= 13)
        {
           if(station == RFID_RX[0]) flag_rx_ok++;
           if(0x03 == RFID_RX[1]) flag_rx_ok++;
           if(0x08 == RFID_RX[2]) flag_rx_ok++;  
           if(flag_rx_ok == 3)
           {
              break;    
           }
        } 
        if (MyTimer_RFID.IsTimeOut())
        {
                           
           break;         
        }
        delay(1);
     }
     if(flag_rx_ok == 3)
     {
        String HEX_0 =  String(RFID_RX[3], HEX); 
        String HEX_1 =  String(RFID_RX[4], HEX); 
        String HEX_2 =  String(RFID_RX[5], HEX); 
        String HEX_3 =  String(RFID_RX[6], HEX); 
        String HEX_4 =  String(RFID_RX[7], HEX); 
        String HEX_5 =  String(RFID_RX[8], HEX); 
        String HEX_6 =  String(RFID_RX[9], HEX); 
        String HEX_7 =  String(RFID_RX[10], HEX); 
        if(HEX_0.length() < 2)HEX_0 = "0" + HEX_0;
        if(HEX_1.length() < 2)HEX_1 = "0" + HEX_1;
        if(HEX_2.length() < 2)HEX_2 = "0" + HEX_2;
        if(HEX_3.length() < 2)HEX_3 = "0" + HEX_3;
        if(HEX_4.length() < 2)HEX_4 = "0" + HEX_4;
        if(HEX_5.length() < 2)HEX_5 = "0" + HEX_5;
        if(HEX_6.length() < 2)HEX_6 = "0" + HEX_6;
        if(HEX_7.length() < 2)HEX_7 = "0" + HEX_7;
        HEX_0.toUpperCase();
        HEX_1.toUpperCase();
        HEX_2.toUpperCase();
        HEX_3.toUpperCase();
        HEX_4.toUpperCase();
        HEX_5.toUpperCase();
        HEX_6.toUpperCase();
        HEX_7.toUpperCase();
        HEX_7 = "00";
        RFID_Error = 0;
        String CardID = HEX_0 + HEX_1 + HEX_2 + HEX_3 + HEX_4 + HEX_5 + HEX_6;
        if(flag_udp_232back)
        {
           printf("station : %d , ID : " ,station);
           mySerial.println(CardID);
        }
        return HEX_0 + HEX_1 + HEX_2 + HEX_3 + HEX_4 + HEX_5 + HEX_6;
     }
     else
     {
        if(flag_udp_232back)printf("station : %d , RFID_len : %d\n" ,station ,RFID_len);
        retry++;
     }
     delay(1);
   }
   return "";   
}
bool Set_Beep(byte station)
{
   int retry = 0;
   byte tx[8];
   tx[0] = station;
   tx[1] = 0x06;
   tx[2] = 0x00;
   tx[3] = 0x04;
   tx[4] = 0x00;
   tx[5] = 0x02;   
   uint16_t CRC = Get_CRC16(tx , 6);
   tx[6] = CRC;
   tx[7] = (CRC >> 8);
   byte flag_rx_ok = 0;
   while(true)
   {
     if(retry >= 2) break;
     flag_rx_ok = 0;
     Set_RS485_Tx_Enable();
     mySerial_485.write(tx , 8);     
     mySerial_485.flush();
     Set_RS485_Rx_Enable();
     RFID_len = 0;
     for (int i = 0 ; i < RFID_RX_SIZE ; i++)
     {
         RFID_RX[i] = 0;         
     }
     MyTimer_RFID.TickStop();
     MyTimer_RFID.StartTickTime(200);
     while(true)
     {
          if (mySerial_485.available())
          {
            RFID_RX[RFID_len] = mySerial_485.read();
            RFID_len++;
            MyTimer_RFID.TickStop();
            MyTimer_RFID.StartTickTime(5);
          }
          if(RFID_len == 8)
          {
             for(int i = 0 ; i < RFID_len ; i++)
             {
                 if(tx[i]== RFID_RX[i])
                 {
                    flag_rx_ok++;
                 }
             }
             if(flag_rx_ok == 8)
             {
                break;
             }
          }  
          if (MyTimer_RFID.IsTimeOut())
          {
                    
            break;
          }
     }
     if(flag_rx_ok == 8)
     {
        if(flag_udp_232back)printf("RFID Set Beep sucess , station : %d \n",station);
        return true;
     }
     else
     {        
        retry++;
     }
   }
   if(flag_udp_232back)printf("RFID Set Beep failed , station : %d \n",station);
   return false;    
}
void Set_RS485_Rx_Enable()
{
    delay(2);
    digitalWrite(PIN_485_Tx_Eanble, LOW);
    delay(2);
}
void Set_RS485_Tx_Enable()
{
    delay(2);
    digitalWrite(PIN_485_Tx_Eanble, HIGH);
    delay(2);
}
uint16_t Get_CRC16(byte* pDataBytes , int len)
{
    uint16_t crc = 0xffff;
    uint16_t polynom = 0xA001;
    for (int i = 0; i < len; i++)
    {
        crc ^= *(pDataBytes + i);
        for (int j = 0; j < 8; j++)
        {
            if ((crc & 0x01) == 0x01)
            {
                crc >>= 1;
                crc ^= polynom;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}
