#include "DAP.h"
#include <ESP8266WiFi.h>
#include <EEPROM.h>

//定义一个写和读的通用方法
#define EEPROM_write(address,p) { int i = 0;byte *pp = (byte*)&(p);for(;i<sizeof(p);i++) EEPROM.write(address+i,pp[i]);EEPROM.end();}
#define EEPROM_read(address,p) { int i = 0;byte *pp = (byte*)&(p);for(;i<sizeof(p);i++) pp[i]=EEPROM.read(address+i);}


byte USART_RX_BUF[1280]={0};     //存储接收到的数据，最多60个数据
byte USART2_RX_BUF[64]={0};     //存储接收到的数据，最多60个数据
byte USART3_RX_BUF[512]={0};     //存储接收到的数据，最多60个数据
byte Response_RX_BUF[64]={0};     //存储接收到的数据，最多60个数据
byte datatemp[8]={0,0,0,0,9,6,0,0};
byte data[8]={0,0,0,0,0,0,0,0};
//接收状态
//bit15，  接收完成标志
//bit14，  接收到0x0d
//bit13~0，  接收到的有效字节数目
word USART_RX_STA=0;       //接收状态标记 
word USART2_RX_STA=0;       //接收状态标记 
word USART3_RX_STA=0;       //接收状态标记 
long baud=0;
byte Res;//保持软串口读取的数据
byte a;

uint16_t Flag_3=0,Flag_4=0,Flag_5=0,D1_len=0,D2_len=0,D3_len=0,len=0,t=0,num = 0;


const char *ssid = "ESP0000";
const char *password = "1234567890";

const IPAddress serverIP(192,168,4,1); //欲访问的地址
uint16_t serverPort = 8080;         //服务器端口号

WiFiClient client; //声明一个客户端对象，用于与服务器进行连接

String str1 = "";
unsigned int num2 = 0;//定义一个变量存储接收到0x7f的次数
unsigned int num3 = 0;//定义一个变量存储接收到0x7f的次数

void setup() 
{
  pinMode(2,OUTPUT);//D1
  digitalWrite(2,HIGH);
  pinMode(14, OUTPUT);//设置IO口14及D5为输出模式
  digitalWrite(14, HIGH);//设置电平为高，导通MOS管
  pinMode(12, OUTPUT);//设置IO口14及D5为输出模式
  digitalWrite(12, HIGH);//设置电平为高，导通MOS管
//  EEPROM.begin(8);//申请内存空间
//  EEPROM_read(0, data);  //从地址0读取数据
//  baud = data[0]*100000+data[1]*10000+data[2]*1000+data[3]*100+data[4]*10+data[5];
//  if(baud%100==0)
//  {
//    EEPROM_write(0, datatemp);
//    baud = 9600;
//  }
  Serial.begin(115200);
  Serial.setTimeout(1);
  //Serial.setRxBufferSize(1024);
  WiFi.mode(WIFI_STA);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
      delay(100);
  }
  delay(100);
  WiFi.disconnect();
}

void loop() 
{
  digitalWrite(2,HIGH);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
  }
  if (client.connect(serverIP, serverPort)) //尝试访问目标地址
  {
    delay(100);
    digitalWrite(2,LOW);
    client.setNoDelay(true);
    while(1)
    {
      if(client.available())
      {
        Res =(byte)client.read(); //读取接收到的数据
        if(((USART_RX_STA&0x8000)||(USART2_RX_STA&0x8000)||(USART3_RX_STA&0x8000))==0)
        {
          if(((USART_RX_STA&0x4000)||(USART2_RX_STA&0x4000)||(USART3_RX_STA&0x4000))==0)
          {
            if(Res==0x66)
            {
              USART_RX_STA|=0x4000;//
              Flag_3 = 1;
            }else if(Res==0x67)
            {
              USART3_RX_STA|=0x4000;//
              Flag_4 = 1;
            }else if(Res==0x68)
            {
              USART2_RX_STA|=0x4000;//
              Flag_5 = 1;
            }else
            {
              USART_RX_STA = 0;
              USART2_RX_STA = 0;
              USART3_RX_STA = 0;
            }
          }else if(USART2_RX_STA&0x4000)
          {
            if(Flag_5)
            {
              D2_len = Res;
              Flag_5 = 0;
            }else
            {
              USART2_RX_BUF[USART2_RX_STA&0X0FFF]=Res ;
              USART2_RX_STA++;
              if((USART2_RX_STA&0x0FFF)==D2_len)USART2_RX_STA|=0x8000;  //
            }
          }else if((((USART_RX_STA&0x4000))&&(~(USART3_RX_STA&0x4000))))
          {
            if(Flag_3)
            {
              D1_len = Res;
              if(D1_len>64)
                D1_len = (D1_len-64)*64;
              Flag_3 = 0;
            }else
            {
              USART_RX_BUF[USART_RX_STA&0X0FFF]=Res ;
              USART_RX_STA++;
              if((USART_RX_STA&0x0FFF)==D1_len)USART_RX_STA|=0x8000;  //
            }
          }else if((((~USART_RX_STA&0x4000))&&((USART3_RX_STA&0x4000))))
          {
            if(Flag_4)
            {
              D3_len = Res;
              Flag_4 = 0;
            }else
            {
              USART3_RX_BUF[USART3_RX_STA&0X0FFF]=Res ;
              USART3_RX_STA++;
              if((USART3_RX_STA&0x0FFF)==D3_len)USART3_RX_STA|=0x8000;  //
            }
          }
        }else if((((USART_RX_STA&0x8000))&&(~(USART3_RX_STA&0x8000))))
        {
          if((USART3_RX_STA&0x4000)==0)
          {
            if(Res==0x67)
            {
              USART3_RX_STA|=0x4000;//
              Flag_4 = 1;
            }else
            {
              USART3_RX_STA = 0;
            }
          }else
          {
            if(Flag_4)
            {
              D3_len = Res;
              Flag_4 = 0;
            }else
            {
              USART3_RX_BUF[USART3_RX_STA&0X0FFF]=Res ;
              USART3_RX_STA++;
              if((USART3_RX_STA&0x0FFF)==D3_len)USART3_RX_STA|=0x8000;  //
            }
          }
        }else if(((~(USART_RX_STA&0x8000))&&((USART3_RX_STA&0x8000))))
        {
          if((USART_RX_STA&0x4000)==0)
          {
            if(Res==0x66)
            {
              USART_RX_STA|=0x4000;//
              Flag_3 = 1;
            }else
            {
              USART_RX_STA = 0;
            }
          }else
          {
            if(Flag_3)
            {
              D1_len = Res;
              if(D1_len>64)
                D1_len = (D1_len-64)*64;
              Flag_3 = 0;
            }else
            {
              USART_RX_BUF[USART_RX_STA&0X0FFF]=Res ;
              USART_RX_STA++;
              if((USART_RX_STA&0x0FFF)==D1_len)USART_RX_STA|=0x8000;  //
            }
          }
        }
      }
      if(Serial.available())
      {
        String s = Serial.readString();
//        if((s.length()==12)&&(s[2]==0x12))
//        {
//            str1 += char(0x67);
//            str1 += char(s.length());
//            str1 += s;
//            delay(2);
//            client.print(str1);
//            str1 = "";
//            delay(2);
//            digitalWrite(12,HIGH);
//        }else
//        {
            str1 += char(0x67);
            str1 += char(s.length());
            str1 += s;
            delay(3);
            client.print(str1);
            str1 = "";
            delay(3);
//        }
      }
      if(USART2_RX_STA&0x8000)
      {
        if(D2_len==7)
        {
          baud = (USART2_RX_BUF[0]-'0')*1000000+(USART2_RX_BUF[1]-'0')*100000+(USART2_RX_BUF[2]-'0')*10000+(USART2_RX_BUF[3]-'0')*1000+(USART2_RX_BUF[4]-'0')*100+(USART2_RX_BUF[5]-'0')*10+(USART2_RX_BUF[6]-'0')*1;
          Serial.begin(baud);
          Serial.setTimeout(1);
        }else if(D2_len==1)
        {
          num2++;
          if(USART2_RX_BUF[0]==2)
          {
            digitalWrite(14, LOW);//拉低IO14，及D5
            digitalWrite(12, LOW);//拉低IO14，及D5
            delay(300);
            digitalWrite(14, HIGH);//拉高IO14，及D5  
            digitalWrite(12, HIGH);//拉低IO14，及D5
            num2=0;
            String s = Serial.readString();
          }
//          Serial.write(USART2_RX_BUF[0]);
//          digitalWrite(12, LOW);//拉低IO14，及D5
//          delayMicroseconds(30);
//          digitalWrite(12, HIGH);//拉高IO14，及D5  
        }
        USART2_RX_STA = 0;
      }
      if(USART_RX_STA&0x8000)
      {
        for(t=0;t<64;t++)USB_Request[0][t] = 0;
        len = D1_len;
        USART_RX_STA = 0;
        if(len>64)
        {
          for(num=0;num<len/64;num++)
          {
            for(t=0;t<64;t++)
              USB_Request[0][t] = USART_RX_BUF[t+num*64];//???1????
            DAP_ExecuteCommand(USB_Request[0],USB_Response[0]);
          }
          str1 += char(0x66);
          str1 += char(0x40);
          for(t=0;t<64;t++)
          {
            str1 += char(USB_Response[0][t]);
          }
          client.print(str1);
          str1 = "";
        }else
        {
          for(t=0;t<D1_len;t++)
            USB_Request[0][t] = USART_RX_BUF[t];//???1????
          DAP_ExecuteCommand(USB_Request[0],USB_Response[0]);
          for(t=63;t>0;t--)
          {
            if(Response_RX_BUF[t] != USB_Response[0][t])break;
          }
          num = t+1;
          str1 += char(0x66);
          str1 += char(num);
          for(t=0;t<num;t++)
          {
            str1 += char(USB_Response[0][t]);
          }
          client.print(str1);
          str1 = "";
        }
        for(t=0;t<64;t++)
          Response_RX_BUF[t] = USB_Response[0][t];
      } 
      if(USART3_RX_STA&0x8000)
      {
        len = D3_len;
        if(len==1)
        {
            if(USART3_RX_BUF[0] == 0x7f)//判断是否为下载合法数据
            {
              if(num2>=2)
              {
                digitalWrite(14, LOW);//拉低IO14，及D5
                delay(100);
                digitalWrite(14, HIGH);//拉高IO14，及D5  
                delay(10);
                String s = Serial.readString();
              }
            }
            Serial.write(USART3_RX_BUF[0]);//软串口发送接收到的数据
            Serial.flush();
        }else if(len==2)
        {
            if(USART3_RX_BUF[0] == 0x30)//判断是否为下载合法数据
            {
              if(USART3_RX_BUF[1] == 0x20)//判断是否为下载合法数据
              {
                if(num2>=2)
                {
                  digitalWrite(14, LOW);//拉低IO14，及D5
                  digitalWrite(12, LOW);//拉低IO14，及D5
                  delay(500);
                  digitalWrite(14, HIGH);//拉高IO14，及D5  
                  digitalWrite(12, HIGH);//拉低IO14，及D5
                  delay(100);
                  String s = Serial.readString();
                  delay(300);
                }
              }
            }
            Serial.write(USART3_RX_BUF[0]);//软串口发送接收到的数据
            Serial.write(USART3_RX_BUF[1]);//软串口发送接收到的数据
            Serial.flush();
//        }else if(len==46)
//        {
//            if(USART3_RX_BUF[2] == 0x08)//判断是否为下载合法数据
//            {
//              num3++;//记录接收到的次数
//            }else
//            {
//              num3 = 0;//清零接收到的次数
//            }
//            for(t=0;t<len;t++)
//            {
//              Serial.write(USART3_RX_BUF[t]);
//            }
//            if((num3>=5)&&(num3<10))//如果接收到的次数在这个范围内就关闭MOS管，给单片机断电
//            {
//              digitalWrite(14, LOW);//拉低IO14，及D5
//              digitalWrite(12,LOW);
//            }else if((num3>=10))//拉低时长超过100*10ms=1s以上后给单片机上电
//            {
//              digitalWrite(14, HIGH);//拉高IO14，及D5
//              num3 = 0;//清空接收到的次数
//            }
//            Serial.flush();
        }else
        {
            for(t=0;t<len;t++)
            {
              Serial.write(USART3_RX_BUF[t]);
            }
            Serial.flush();
        }
        num2 = 0;//清空接收到的次数
        USART3_RX_STA = 0;
      }
      if(WiFi.isConnected()==0)
      {
        digitalWrite(LED_BUILTIN, 1);//灭
        for(t=0;t<64;t++)
          Response_RX_BUF[t] = 0;
        D1_len = 0;
        ESP.restart();
      }
    }
  }
}

//void softSerial1_Handle()
//{
//  if(softSerial1.available()>0)    //接收串口屏代码
//  {
//    Res =(unsigned char)softSerial1.read(); //读取接收到的数据
//    if((USART_RX_STA2&0x8000)==0)//接收未完成
//    {
//      if(USART_RX_STA2&0x4000)//接收到了0x0d
//      {
//        if(Res!=0x0a)
//        {
//          USART_RX_STA2&=0xBFFF;//接收错误,重新开始
//          if(Res==0x0d)USART_RX_STA2|=0x4000;
//          USART_RX_BUF2[USART_RX_STA2&0X3FFF]=Res ;
//          USART_RX_STA2++;
//        }else 
//        {
//          USART_RX_STA2|=0x8000;  //接收完成了 
//          USART_RX_BUF2[USART_RX_STA2&0X3FFF]=Res ;
//          USART_RX_STA2++;
//          BMQ1=USART_RX_BUF[0];BMQ1=BMQ1<<8;BMQ1|=USART_RX_BUF[1];
//          BMQ2=USART_RX_BUF[2];BMQ2=BMQ2<<8;BMQ2|=USART_RX_BUF[3];
//          BMQ3=USART_RX_BUF[4];BMQ3=BMQ3<<8;BMQ3|=USART_RX_BUF[5];
//          BMQ4=USART_RX_BUF[6];BMQ4=BMQ4<<8;BMQ4|=USART_RX_BUF[7];
//          BMQ5=USART_RX_BUF[8];BMQ5=BMQ5<<8;BMQ5|=USART_RX_BUF[9];
//          USART_RX_STA2=0;
//        }
//      }else //还没收到0X0D
//      { 
//        if(Res==0x0d)
//        {
//          USART_RX_STA2|=0x4000;
//          USART_RX_BUF2[USART_RX_STA2&0X3FFF]=Res ;
//          USART_RX_STA2++;
//        }else
//        {
//          USART_RX_BUF2[USART_RX_STA2&0X3FFF]=Res ;
//          USART_RX_STA2++;
//        }    
//      }
//    }
//  }
//}
