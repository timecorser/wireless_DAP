#ifndef __DAP_H__
#define __DAP_H__

#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <stdint.h>

// DAP的控制ID
#define ID_DAP_Info          0x00
#define ID_DAP_LED          0x01
#define ID_DAP_Connect        0x02
#define ID_DAP_Disconnect     0x03
#define ID_DAP_TransferConfigure  0x04
#define ID_DAP_Transfer       0x05
#define ID_DAP_TransferBlock    0x06
#define ID_DAP_TransferAbort    0x07
#define ID_DAP_WriteABORT     0x08
#define ID_DAP_Delay        0x09
#define ID_DAP_ResetTarget      0x0A
#define ID_DAP_SWJ_Pins       0x10
#define ID_DAP_SWJ_Clock      0x11
#define ID_DAP_SWJ_Sequence     0x12
#define ID_DAP_SWD_Configure    0x13
#define ID_DAP_JTAG_Sequence    0x14
#define ID_DAP_JTAG_Configure   0x15
#define ID_DAP_JTAG_IDCODE      0x16

//DAP的主机控制ID
#define ID_DAP_Vendor0        0x80
#define ID_DAP_Vendor1        0x81
#define ID_DAP_Vendor2        0x82
#define ID_DAP_Vendor3        0x83
#define ID_DAP_Vendor4        0x84
#define ID_DAP_Vendor5        0x85
#define ID_DAP_Vendor6        0x86
#define ID_DAP_Vendor7        0x87
#define ID_DAP_Vendor8        0x88
#define ID_DAP_Vendor9        0x89
#define ID_DAP_Vendor10       0x8A
#define ID_DAP_Vendor11       0x8B
#define ID_DAP_Vendor12       0x8C
#define ID_DAP_Vendor13       0x8D
#define ID_DAP_Vendor14       0x8E
#define ID_DAP_Vendor15       0x8F
#define ID_DAP_Vendor16       0x90
#define ID_DAP_Vendor17       0x91
#define ID_DAP_Vendor18       0x92
#define ID_DAP_Vendor19       0x93
#define ID_DAP_Vendor20       0x94
#define ID_DAP_Vendor21       0x95
#define ID_DAP_Vendor22       0x96
#define ID_DAP_Vendor23       0x97
#define ID_DAP_Vendor24       0x98
#define ID_DAP_Vendor25       0x99
#define ID_DAP_Vendor26       0x9A
#define ID_DAP_Vendor27       0x9B
#define ID_DAP_Vendor28       0x9C
#define ID_DAP_Vendor29       0x9D
#define ID_DAP_Vendor30       0x9E
#define ID_DAP_Vendor31       0x9F

#define ID_DAP_Invalid        0xFF

//DAP的状态码
#define DAP_OK            0
#define DAP_ERROR         0xFF

// DAP的ID种类
#define DAP_ID_VENDOR       1
#define DAP_ID_PRODUCT        2
#define DAP_ID_SER_NUM        3
#define DAP_ID_FW_VER       4
#define DAP_ID_DEVICE_VENDOR    5
#define DAP_ID_DEVICE_NAME      6
#define DAP_ID_CAPABILITIES     0xF0
#define DAP_ID_PACKET_COUNT     0xFE
#define DAP_ID_PACKET_SIZE      0xFF

// DAP的LED
#define DAP_LED_DEBUGGER_CONNECTED  0
#define DAP_LED_TARGET_RUNNING    1

// DAP的下载方式
#define DAP_PORT_AUTODETECT     0   // Autodetect Port
#define DAP_PORT_DISABLED     0   // Port Disabled (I/O pins in High-Z)
#define DAP_PORT_SWD        1   // SWD Port (SWCLK, SWDIO) + nRESET
#define DAP_PORT_JTAG       2   // JTAG Port (TCK, TMS, TDI, TDO, nTRST) + nRESET

// DAP SWJ方式的引脚
#define DAP_SWJ_SWCLK_TCK     0   // SWCLK/TCK
#define DAP_SWJ_SWDIO_TMS     1   // SWDIO/TMS
#define DAP_SWJ_TDI         2   // TDI
#define DAP_SWJ_TDO         3   // TDO
#define DAP_SWJ_nTRST       5   // nTRST
#define DAP_SWJ_nRESET        7   // nRESET

// DAP的传递请求
#define DAP_TRANSFER_APnDP      (1 << 0)
#define DAP_TRANSFER_RnW      (1 << 1)
#define DAP_TRANSFER_A2       (1 << 2)
#define DAP_TRANSFER_A3       (1 << 3)
#define DAP_TRANSFER_MATCH_VALUE  (1 << 4)
#define DAP_TRANSFER_MATCH_MASK   (1 << 5)

// DAP的传递回复
#define DAP_TRANSFER_OK       (1 << 0)
#define DAP_TRANSFER_WAIT     (1 << 1)
#define DAP_TRANSFER_FAULT      (1 << 2)
#define DAP_TRANSFER_ERROR      (1 << 3)
#define DAP_TRANSFER_MISMATCH   (1 << 4)

//调试端口寄存器地址
#define DP_IDCODE         0x00  // IDCODE Register (SW Read only)
#define DP_ABORT          0x00  // Abort Register (SW Write only)
#define DP_CTRL_STAT        0x04  // Control & Status
#define DP_WCR            0x04  // Wire Control Register (SW Only)
#define DP_SELECT         0x08  // Select Register (JTAG R/W & SW W)
#define DP_RESEND         0x08  // Resend (SW Read Only)
#define DP_RDBUFF         0x0C  // Read Buffer (Read Only)

//可配置的时钟生成延迟
#define DELAY_SLOW_CYCLES   6 // Number of cycles for one iteration

//固定的快速时钟生成
#define DELAY_FAST_CYCLES   1 // Number of cycles

//DAP数据结构
typedef struct
{
  byte  debug_port;      // Debug Port
  byte  fast_clock;      // Fast Clock Flag
  unsigned long  clock_delay;    // Clock Delay
  struct {            // Transfer Configuration
    byte idle_cycles;    // Idle cycles after transfer
    word  retry_count;    // Number of retries after WAIT response
    word  match_retry;    // Number of retries if read value does not match
    unsigned long  match_mask;   // Match Mask
  } transfer;
  struct {            // SWD Configuration
    byte   turnaround;   // Turnaround period
    byte   data_phase;   // Always generate Data Phase
  } swd_conf;
} DAP_Data_t;

//调试单元中使用的处理器时钟，用于计算SWD的时钟速率
#define CPU_CLOCK       80000000   ///< Specifies the CPU Clock in Hz

//IO端口写操作的处理器周期数，计算IO口生成的SWD时钟速度
#define IO_PORT_WRITE_CYCLES  4               ///< I/O Cycles: 2=default, 1=Cortex-M0+ fast I/0

//调试访问端口的默认通信模式，当选择“端口默认模式”时，用于DAP_Connect命令
#define DAP_DEFAULT_PORT        1               ///< Default JTAG/SWJ Port Mode: 1 = SWD, 2 = JTAG.

//SWD默认下载速度
#define DAP_DEFAULT_SWJ_CLOCK   100000         ///< Default SWD/JTAG clock frequency in Hz.

//收发数据包的大小
#define DAP_PACKET_SIZE     64        ///< USB: 64 = Full-Speed, 1024 = High-Speed.

//收发数据包缓存区的大小
#define DAP_PACKET_COUNT    1       ///< Buffers: 64 = Full-Speed, 4 = High-Speed.

extern byte  USB_Request [DAP_PACKET_COUNT][DAP_PACKET_SIZE];  // Request  Buffer
extern byte  USB_Response[DAP_PACKET_COUNT][DAP_PACKET_SIZE]; 

#define LED  2

// SWDIO/TMS Pin
#define PIN_SWDIO_TMS_PIN   5//2//D5

// SWCLK/TCK Pin
#define PIN_SWCLK_TCK_PIN   4//0//D6

// nRESET Pin
#define PIN_nRESET_PIN      13


#define PIN_nRESET_LOW() do{pinMode(PIN_nRESET_PIN, OUTPUT);digitalWrite(PIN_nRESET_PIN,LOW);}while(0)
#define PIN_nRESET_HIGH() do{pinMode(PIN_nRESET_PIN, OUTPUT);digitalWrite(PIN_nRESET_PIN,HIGH);}while(0)

//快速切换输入输出模式
#define PIN_SWDIO_TMS_OUT_DISABLE() pinMode(PIN_SWDIO_TMS_PIN, INPUT)
#define PIN_SWDIO_TMS_OUT_ENABLE() pinMode(PIN_SWDIO_TMS_PIN, OUTPUT);
    
#define DAP_SETUP() PORT_OFF()
#define PIN_SWCLK_SET()   PIN_SWCLK_TCK_SET()
#define PIN_SWCLK_CLR()   PIN_SWCLK_TCK_CLR()
//时钟一个周期
#define SW_CLOCK_CYCLE()  do{PIN_SWCLK_CLR();PIN_DELAY();PIN_SWCLK_SET();PIN_DELAY();}while(0)
//写一位
#define SW_WRITE_BIT(sbit)  do{PIN_SWDIO_OUT(sbit);PIN_SWCLK_CLR();PIN_DELAY();PIN_SWCLK_SET();PIN_DELAY();}while(0)
//读一位
#define SW_READ_BIT(sbit)   do{PIN_SWCLK_CLR();PIN_DELAY();sbit = PIN_SWDIO_IN();PIN_SWCLK_SET();PIN_DELAY();}while(0)

//时钟一个周期
#define SWF_CLOCK_CYCLE()  do{PIN_SWCLK_CLR();PIN_SWCLK_SET();}while(0)
//写一位
#define SWF_WRITE_BIT(sbit)  do{PIN_SWDIO_OUT(sbit);PIN_SWCLK_CLR();PIN_SWCLK_SET();}while(0)
//读一位
#define SWF_READ_BIT(sbit)   do{PIN_SWCLK_CLR();sbit = PIN_SWDIO_IN();PIN_SWCLK_SET();}while(0)

#define PIN_DELAY()   PIN_DELAY_SLOW(DAP_Data.clock_delay)

extern DAP_Data_t DAP_Data;     // DAP Data
extern volatile byte  DAP_TransferAbort;  // Transfer Abort Flag

//函数
//extern void PIN_DELAY_SLOW (unsigned long _delay);
extern void PORT_SWD_SETUP(void);
extern void PORT_OFF(void);
extern byte PIN_SWCLK_TCK_IN(void);
extern void PIN_SWCLK_TCK_SET(void);
extern void PIN_SWCLK_TCK_CLR (void);
extern byte PIN_SWDIO_TMS_IN(void);
extern void PIN_SWDIO_TMS_SET(void);
extern void PIN_SWDIO_TMS_CLR(void);
extern byte PIN_SWDIO_IN (void);
extern void PIN_SWDIO_OUT(byte sbit);
extern void PIN_SWDIO_OUT_ENABLE(void);
extern void PIN_SWDIO_OUT_DISABLE(void);
extern byte PIN_nTRST_IN(void);
extern void PIN_nTRST_OUT(byte sbit);
extern byte PIN_nRESET_IN(void);
extern byte RESET_TARGET(void);
extern byte RST_Transfer(unsigned long request, unsigned long data);
extern void PIN_nRESET_OUT(byte sbit);
//extern byte DAP_Info(byte id, byte *_info);
//extern unsigned long DAP_Delay(byte *request, byte *response);
//extern unsigned long DAP_LED(byte *request, byte *response);
//extern unsigned long DAP_Connect(byte *request, byte *response);
//extern unsigned long DAP_Disconnect(byte *response);
//extern unsigned long DAP_ResetTarget(byte *response);
//extern unsigned long DAP_SWJ_Pins(byte *request, byte *response);
//extern unsigned long DAP_SWJ_Clock(byte *request, byte *response);
//extern unsigned long DAP_SWJ_Sequence(byte *request, byte *response);
//extern unsigned long DAP_SWD_Configure(byte *request, byte *response);
//extern unsigned long DAP_SWD_Abort(byte *request, byte *response);
//extern unsigned long DAP_TransferConfigure(byte *request, byte *response);
//extern unsigned long DAP_SWD_Transfer(byte *request, byte *response);
//extern unsigned long DAP_SWD_TransferBlock(byte *request, byte *response);
extern unsigned long DAP_ProcessVendorCommand(byte *request, byte *response);
extern unsigned long DAP_ProcessCommand(byte *request, byte *response);
extern void DAP_Setup(void);
extern void SWJ_Sequence (unsigned long count, byte *data);
extern byte SWD_TransferSlow(byte request, unsigned long *data);
extern byte SWD_Transfer(byte request, unsigned long *data);
extern unsigned long DAP_ExecuteCommand(byte *request, byte *response);

#define DAP_FW_VER      "1.0"   //固件版本

//时钟宏定义
#define MAX_SWJ_CLOCK(delay_cycles)  ( CPU_CLOCK / 2 / (IO_PORT_WRITE_CYCLES + delay_cycles))
#define CLOCK_DELAY(swj_clock)    ((CPU_CLOCK / 2 / swj_clock) - IO_PORT_WRITE_CYCLES)

DAP_Data_t DAP_Data;           // DAP Data
volatile byte DAP_TransferAbort;  // Trasfer Abort Flag

static const char DAP_FW_Ver [] = DAP_FW_VER;
byte  USB_Request[DAP_PACKET_COUNT][DAP_PACKET_SIZE];  // Request  Buffer
byte  USB_Response[DAP_PACKET_COUNT][DAP_PACKET_SIZE]; 

static void PIN_DELAY_SLOW (unsigned long _delay)
{
  delayMicroseconds(_delay);
}

//配置SWD引脚
void PORT_SWD_SETUP()
{
  //推挽输出
  pinMode(PIN_SWDIO_TMS_PIN, OUTPUT);
  digitalWrite(PIN_SWDIO_TMS_PIN,HIGH);
  // Set SWDIO HIGH
  //推挽输出
  pinMode(PIN_SWCLK_TCK_PIN, OUTPUT);
  digitalWrite(PIN_SWCLK_TCK_PIN,HIGH);
  // Set RESET HIGH
  //推挽输出
  pinMode(PIN_nRESET_PIN, OUTPUT);
  digitalWrite(PIN_nRESET_PIN,HIGH);
}

//复位SWD引脚
void PORT_OFF()
{
  pinMode(PIN_SWDIO_TMS_PIN, INPUT);
  pinMode(PIN_SWCLK_TCK_PIN, INPUT);
  pinMode(PIN_nRESET_PIN, INPUT);
}

//读取clk引脚电平
byte PIN_SWCLK_TCK_IN(void)
{
  if (digitalRead(PIN_SWCLK_TCK_PIN))
    return 1;
  return 0;
}

//拉高CLK
void PIN_SWCLK_TCK_SET(void)
{
  digitalWrite(PIN_SWCLK_TCK_PIN,HIGH);
}

//拉低CLK
void PIN_SWCLK_TCK_CLR (void)
{
  digitalWrite(PIN_SWCLK_TCK_PIN,LOW);
}

//读取DIO电平
byte PIN_SWDIO_TMS_IN(void)
{
  if (digitalRead(PIN_SWDIO_TMS_PIN))
    return 1;
  return 0;
}

//拉高DIO
void PIN_SWDIO_TMS_SET(void)
{
  digitalWrite(PIN_SWDIO_TMS_PIN,HIGH);
}

//拉低DIO
void PIN_SWDIO_TMS_CLR(void)
{
  digitalWrite(PIN_SWDIO_TMS_PIN,LOW);
}

//读取DIO引脚状态
byte PIN_SWDIO_IN (void)
{
  if (digitalRead(PIN_SWDIO_TMS_PIN))
    return 1;
  return 0;
}

//DIO写1位
void PIN_SWDIO_OUT(byte sbit)
{
  if (sbit & 1)
    digitalWrite(PIN_SWDIO_TMS_PIN,HIGH);
  else
    digitalWrite(PIN_SWDIO_TMS_PIN,LOW);
}

//DIO切换到输出模式
void PIN_SWDIO_OUT_ENABLE(void)
{
  PIN_SWDIO_TMS_OUT_ENABLE();
}

//DIO切换到输入模式
void PIN_SWDIO_OUT_DISABLE(void)
{
  PIN_SWDIO_TMS_OUT_DISABLE();
}

//读取引脚状态
byte PIN_nTRST_IN(void)
{
  return (0);   // Not available
}

//引脚置1或0
void PIN_nTRST_OUT(byte sbit)
{

}

//读取引脚状态
byte PIN_nRESET_IN(void)
{
  if (digitalRead(PIN_nRESET_PIN))
    return 1;
  return 0;
}

//特定的复位序列重置目标设备
byte RESET_TARGET(void)
{
  return (0); // change to '1' when a device reset sequence is implemented
}

byte RST_Transfer(unsigned long request, unsigned long data)
{
  unsigned long ack;                                                                 
  unsigned long sbit;                                                                 
  unsigned long val;                                                                 
  unsigned long parity;                                                              
  unsigned long n;                                                                   
  if(DAP_Data.fast_clock)
  {
    /* Packet Request */                                                          
    parity = 0U;                                                                  
    SWF_WRITE_BIT(1U);                     /* Start Bit */                        
    sbit = request >> 0;                                                           
    SWF_WRITE_BIT(sbit);                    /* APnDP Bit */                        
    parity += sbit;                                                                
    sbit = request >> 1;                                                           
    SWF_WRITE_BIT(sbit);                    /* RnW Bit */                          
    parity += sbit;                                                                
    sbit = request >> 2;                                                           
    SWF_WRITE_BIT(sbit);                    /* A2 Bit */                           
    parity += sbit;                                                                
    sbit = request >> 3;                                                           
    SWF_WRITE_BIT(sbit);                    /* A3 Bit */                           
    parity += sbit;                                                                
    SWF_WRITE_BIT(parity);                 /* Parity Bit */                       
    SWF_WRITE_BIT(0U);                     /* Stop Bit */                         
    SWF_WRITE_BIT(1U);                     /* Park Bit */                         
    
    /* Turnaround */                                                              
    PIN_SWDIO_OUT_DISABLE();                                                      
    for (n = DAP_Data.swd_conf.turnaround; n>0; n--) 
    {                              
      SWF_CLOCK_CYCLE();                                                          
    }                                                                             
    /* Acknowledge response */                                                    
    SWF_READ_BIT(sbit);                                                            
    ack  = sbit << 0;                                                              
    SWF_READ_BIT(sbit);                                                            
    ack |= sbit << 1;                                                              
    SWF_READ_BIT(sbit);                                                            
    ack |= sbit << 2;                                                              
  
    /* Data transfer */                                                           
    /* Turnaround */                                                              
    for (n = DAP_Data.swd_conf.turnaround; n>0; n--) 
    {                              
      SWF_CLOCK_CYCLE();                                                          
    }                                                                             
    PIN_SWDIO_OUT_ENABLE();                                                       
    /* Write data */                                                              
    val = data;                                                                   
    parity = 0U;                                                                  
    for (n = 32U; n>0; n--) 
    {                                                       
      SWF_WRITE_BIT(val);              /* Write WDATA[0:31] */                    
      parity += val;                                                              
      val >>= 1;                                                                  
    }                                                                             
    SWF_WRITE_BIT(parity);             /* Write Parity Bit */                     
    PIN_SWDIO_OUT_ENABLE();                                                       
    PIN_SWDIO_OUT(1U);                
  }else
  {
      /* Packet Request */                                                          
    parity = 0U;                                                                  
    SW_WRITE_BIT(1U);                     /* Start Bit */                        
    sbit = request >> 0;                                                           
    SW_WRITE_BIT(sbit);                    /* APnDP Bit */                        
    parity += sbit;                                                                
    sbit = request >> 1;                                                           
    SW_WRITE_BIT(sbit);                    /* RnW Bit */                          
    parity += sbit;                                                                
    sbit = request >> 2;                                                           
    SW_WRITE_BIT(sbit);                    /* A2 Bit */                           
    parity += sbit;                                                                
    sbit = request >> 3;                                                           
    SW_WRITE_BIT(sbit);                    /* A3 Bit */                           
    parity += sbit;                                                                
    SW_WRITE_BIT(parity);                 /* Parity Bit */                       
    SW_WRITE_BIT(0U);                     /* Stop Bit */                         
    SW_WRITE_BIT(1U);                     /* Park Bit */                         
    
    /* Turnaround */                                                              
    PIN_SWDIO_OUT_DISABLE();                                                      
    for (n = DAP_Data.swd_conf.turnaround; n>0; n--) 
    {                              
      SW_CLOCK_CYCLE();                                                          
    }                                                                             
    /* Acknowledge response */                                                    
    SW_READ_BIT(sbit);                                                            
    ack  = sbit << 0;                                                              
    SW_READ_BIT(sbit);                                                            
    ack |= sbit << 1;                                                              
    SW_READ_BIT(sbit);                                                            
    ack |= sbit << 2;                                                              
  
    /* Data transfer */                                                           
    /* Turnaround */                                                              
    for (n = DAP_Data.swd_conf.turnaround; n>0; n--) 
    {                              
      SW_CLOCK_CYCLE();                                                          
    }                                                                             
    PIN_SWDIO_OUT_ENABLE();                                                       
    /* Write data */                                                              
    val = data;                                                                   
    parity = 0U;                                                                  
    for (n = 32U; n>0; n--) 
    {                                                       
      SW_WRITE_BIT(val);              /* Write WDATA[0:31] */                    
      parity += val;                                                              
      val >>= 1;                                                                  
    }                                                                             
    SW_WRITE_BIT(parity);             /* Write Parity Bit */                     
    PIN_SWDIO_OUT_ENABLE();                                                       
    PIN_SWDIO_OUT(1U);                
  }
                                              
  return ((byte)ack);                                                        
}

void PIN_nRESET_OUT(byte sbit)
{
  unsigned long i;
  //soft-reset for Cortex-M
  RST_Transfer(0x00000CC5, 0xE000ED0C); //set AIRCR address
  for (i=0; i<100; i++);
  RST_Transfer(0x00000CDD, 0x05FA0007); //set RESET data
  for (i=0; i<100; i++);
  
  if(sbit&1)  
    PIN_nRESET_HIGH();
  else          
    PIN_nRESET_LOW();
}

//获取DAP信息
static byte DAP_Info(byte id, byte *_info)
{
  byte _length = 0;

  switch (id)
  {
    case DAP_ID_VENDOR:
      break;
    case DAP_ID_PRODUCT:
      break;
    case DAP_ID_SER_NUM:
      break;
    case DAP_ID_FW_VER:
      memcpy(_info, DAP_FW_Ver, sizeof(DAP_FW_Ver));
      _length = sizeof(DAP_FW_Ver);
      break;
    case DAP_ID_DEVICE_VENDOR:
      break;
    case DAP_ID_DEVICE_NAME:
      break;
    case DAP_ID_CAPABILITIES:
      _info[0] = ((1  != 0) ? (1 << 0) : 0)|((0 != 0) ? (1 << 1) : 0);
      _length = 1;
      break;
    case DAP_ID_PACKET_SIZE:
      _info[0] = (byte)(DAP_PACKET_SIZE >> 0);
      _info[1] = (byte)(DAP_PACKET_SIZE >> 8);
      _length = 2;
      break;
    case DAP_ID_PACKET_COUNT:
      _info[0] = DAP_PACKET_COUNT;
      _length = 1;
      break;
  }

  return (_length);
}

//进程延迟命令和准备响应
static unsigned long DAP_Delay(byte *request, byte *response) 
{
  unsigned long _delay;

  _delay  = *(request + 0) | (*(request + 1) << 8);
  _delay *= (CPU_CLOCK / 1000000 + (DELAY_SLOW_CYCLES-1)) / DELAY_SLOW_CYCLES;

  PIN_DELAY_SLOW(_delay);

  *response = DAP_OK;
  return (1);
}

//处理LED指令
static unsigned long DAP_LED(byte *request, byte *response)
{
  switch (*request)
  {
    case DAP_LED_DEBUGGER_CONNECTED:
//      LED_CONNECTED_OUT((*(request + 1) & 1));
      break;
    case DAP_LED_TARGET_RUNNING:
//      LED_RUNNING_OUT((*(request + 1) & 1));
      break;
    default:
      *response = DAP_ERROR;
      return (1);
  }
  *response = DAP_OK;
  return (1);
}

//处理连接命令
static unsigned long DAP_Connect(byte *request, byte *response)
{
  unsigned long port;

  if (*request == DAP_PORT_AUTODETECT)
  {
    port = DAP_DEFAULT_PORT;
  }
  else
  {
    port = *request;
  }

  switch (port)
  {
    case DAP_PORT_SWD:
      DAP_Data.debug_port = DAP_PORT_SWD;
      PORT_SWD_SETUP();
      break;
    default:
      *response = DAP_PORT_DISABLED;
      return (1);
  }

  *response = port;
  return (1);
}

//处理断开连接指令
static unsigned long DAP_Disconnect(byte *response)
{
  DAP_Data.debug_port = DAP_PORT_DISABLED;
  PORT_OFF();

  *response = DAP_OK;
  return (1);
}

//处理复位项目指令
static unsigned long DAP_ResetTarget(byte *response)
{
  *(response + 1) = RESET_TARGET();
  *(response + 0) = DAP_OK;
  return (2);
}

//处理SW引脚命令
static unsigned long DAP_SWJ_Pins(byte *request, byte *response)
{
  byte value;
  byte select;
  unsigned long wait;
  
  value =    *(request + 0);
  select  =  *(request + 1);
  wait  = (*(request + 2) << 0)|(*(request + 3) << 8)|(*(request + 4) << 16)|(*(request + 5) << 24);

  if (select & (1 << DAP_SWJ_SWCLK_TCK))
  {
    if (value & (1 << DAP_SWJ_SWCLK_TCK))
      PIN_SWCLK_TCK_SET();
    else
      PIN_SWCLK_TCK_CLR();
  }

  if (select & (1 << DAP_SWJ_SWDIO_TMS))
  {
    if (value & (1 << DAP_SWJ_SWDIO_TMS))
      PIN_SWDIO_TMS_SET();
    else
      PIN_SWDIO_TMS_CLR();
  }


  if (select & (1 << DAP_SWJ_nTRST))
    PIN_nTRST_OUT(value >> DAP_SWJ_nTRST);

  if (select & (1 << DAP_SWJ_nRESET))
    PIN_nRESET_OUT(value >> DAP_SWJ_nRESET);

  if (wait)
  {
    if (wait > 3000000)
      wait = 3000000;
    do 
    {
      delayMicroseconds(1);
      if(wait>0)
      wait--;
      if (select & (1 << DAP_SWJ_SWCLK_TCK))
      {
        if ((value >> DAP_SWJ_SWCLK_TCK) ^ PIN_SWCLK_TCK_IN())
          continue;
      }
      if (select & (1 << DAP_SWJ_SWDIO_TMS))
      {
        if ((value >> DAP_SWJ_SWDIO_TMS) ^ PIN_SWDIO_TMS_IN())
          continue;
      }
      if (select & (1 << DAP_SWJ_nTRST))
      {
        if ((value >> DAP_SWJ_nTRST) ^ PIN_nTRST_IN())
          continue;
      }
      if (select & (1 << DAP_SWJ_nRESET))
      {
        if ((value >> DAP_SWJ_nRESET) ^ PIN_nRESET_IN())
          continue;
      }
      break;
    } while (wait);
  }

  value = (PIN_SWCLK_TCK_IN()<<DAP_SWJ_SWCLK_TCK)|(PIN_SWDIO_TMS_IN()<<DAP_SWJ_SWDIO_TMS)|(PIN_nTRST_IN()<< DAP_SWJ_nTRST)|(PIN_nRESET_IN()<< DAP_SWJ_nRESET);

  *response = (byte)value;
  return (1);
}

//处理SW时钟指令
static unsigned long DAP_SWJ_Clock(byte *request, byte *response)
{
  unsigned long _clock;
  unsigned long _delay;

  _clock = (*(request + 0)<<0)|(*(request + 1)<<8)|(*(request + 2)<<16)|(*(request + 3)<<24);

  if (_clock == 0)
  {
    *response = DAP_ERROR;
    return (1);
  }

  if (_clock > 500000)
  {
    DAP_Data.fast_clock  = 1;
    DAP_Data.clock_delay = 1;
  }
  else
  {
    DAP_Data.fast_clock  = 0;

    _delay = 500000 / _clock;
    DAP_Data.clock_delay = _delay;
  }

  *response = DAP_OK;
  return (1);
}

//处理SW序列
static unsigned long DAP_SWJ_Sequence(byte *request, byte *response)
{
  unsigned long count;

  count = *request++;
  if (count == 0)
    count = 256;

  SWJ_Sequence(count, request);

  *response = DAP_OK;
  return (1);
}


//处理SW配置指令
static unsigned long DAP_SWD_Configure(byte *request, byte *response)
{
  byte value;

  value = *request;
  DAP_Data.swd_conf.turnaround  = (value & 0x03) + 1;
  DAP_Data.swd_conf.data_phase  = (value & 0x04) ? 1 : 0;
  
  *response = DAP_OK;
  return (1);
}

//处理SW中止指令
static unsigned long DAP_SWD_Abort(byte *request, byte *response)
{
  unsigned long data;
  
  if (DAP_Data.debug_port != DAP_PORT_SWD)
  {
    *response = DAP_ERROR;
    return (1);
  }

  // Load data (Ignore DAP index)
  data =  (*(request+1) <<  0) |(*(request+2) <<  8) |(*(request+3) << 16) |(*(request+4) << 24);
  // Write Abort register
  SWD_Transfer(DP_ABORT, &data);
  *response = DAP_OK;

  return (1);
}

//处理转移配置指令
static unsigned long DAP_TransferConfigure(byte *request, byte *response)
{
  DAP_Data.transfer.idle_cycles = *(request + 0);
  DAP_Data.transfer.retry_count = *(request + 1) | (*(request + 2) << 8);
  DAP_Data.transfer.match_retry = *(request + 3) | (*(request + 4) << 8);
  *response = DAP_OK;
  return (1);
}

//处理SW转移指令
static unsigned long DAP_SWD_Transfer(byte *request, byte *response)
{
  byte  request_count;
  byte  request_value;
  byte  response_count;
  byte  response_value;
  byte  *response_head;
  unsigned long  post_read;
  unsigned long  check_write;
  unsigned long  match_value;
  word  match_retry;
  word  retry;
  unsigned long  data;
  
  response_count = 0;
  response_value = 0;
  response_head  = response;
  response += 2;

  DAP_TransferAbort = 0;

  post_read   = 0;
  check_write = 0;

  request++;            // Ignore DAP index

  request_count = *request++;

  while (request_count--)
  {
    request_value = *request++;
    if (request_value & DAP_TRANSFER_RnW)
    {
      // Read register
      if (post_read)
      {
        // Read was posted before
        retry = DAP_Data.transfer.retry_count;
        if ((request_value & (DAP_TRANSFER_APnDP | DAP_TRANSFER_MATCH_VALUE)) == DAP_TRANSFER_APnDP)
        {
          // Read previous AP data and post next AP read
          do
          {
            response_value = SWD_Transfer(request_value, &data);
          } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
        }
        else
        {
          // Read previous AP data
          do
          {
            response_value = SWD_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, &data);
          } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
          post_read = 0;
        }
        if (response_value != DAP_TRANSFER_OK)
          break;
        // Store previous AP data
        *response++ = (byte) data;
        *response++ = (byte)(data >>  8);
        *response++ = (byte)(data >> 16);
        *response++ = (byte)(data >> 24);
      }
      if (request_value & DAP_TRANSFER_MATCH_VALUE)
      {
        // Read with value match
        match_value = (*(request+0)<<0)|(*(request+1)<<8)|(*(request+2)<<16)|(*(request+3)<<24);
        request += 4;
        match_retry = DAP_Data.transfer.match_retry;
        if (request_value & DAP_TRANSFER_APnDP)
        {
          // Post AP read
          retry = DAP_Data.transfer.retry_count;
          do
          {
            response_value = SWD_Transfer(request_value, NULL);
          } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
          if (response_value != DAP_TRANSFER_OK) break;
        }
        do
        {
          // Read register until its value matches or retry counter expires
          retry = DAP_Data.transfer.retry_count;
          do
          {
            response_value = SWD_Transfer(request_value, &data);
          } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
          
          if (response_value != DAP_TRANSFER_OK)
            break;
        } while (((data & DAP_Data.transfer.match_mask) != match_value) && match_retry-- && !DAP_TransferAbort);
        if ((data & DAP_Data.transfer.match_mask) != match_value)
        {
          response_value |= DAP_TRANSFER_MISMATCH;
        }
        if (response_value != DAP_TRANSFER_OK) break;
      }
      else
      {
        // Normal read
        retry = DAP_Data.transfer.retry_count;
        if (request_value & DAP_TRANSFER_APnDP)
        {
          // Read AP register
          if (post_read == 0)
          {
            // Post AP read
            do
            {
              response_value = SWD_Transfer(request_value, NULL);
            } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
            if (response_value != DAP_TRANSFER_OK) break;
            post_read = 1;
          }
        }
        else
        {
          // Read DP register
          do
          {
            response_value = SWD_Transfer(request_value, &data);
          } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
          if (response_value != DAP_TRANSFER_OK) break;
          // Store data
          *response++ = (byte) data;
          *response++ = (byte)(data >>  8);
          *response++ = (byte)(data >> 16);
          *response++ = (byte)(data >> 24);
        }
      }
      check_write = 0;
    }
    else
    {
      // Write register
      if (post_read)
      {
        // Read previous data
        retry = DAP_Data.transfer.retry_count;
        do
        {
          response_value = SWD_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, &data);
        } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
        
        if (response_value != DAP_TRANSFER_OK)
          break;
        // Store previous data
        *response++ = (byte) data;
        *response++ = (byte)(data >>  8);
        *response++ = (byte)(data >> 16);
        *response++ = (byte)(data >> 24);
        post_read = 0;
      }
      // Load data
      data =  (*(request+0) <<  0)|(*(request+1) <<  8)|(*(request+2) << 16)|(*(request+3) << 24);
      request += 4;
      if (request_value & DAP_TRANSFER_MATCH_MASK)
      {
        // Write match mask
        DAP_Data.transfer.match_mask = data;
        response_value = DAP_TRANSFER_OK;
      }
      else
      {
        // Write DP/AP register
        retry = DAP_Data.transfer.retry_count;
        do
        {
          response_value = SWD_Transfer(request_value, &data);
        } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
        
        if (response_value != DAP_TRANSFER_OK)
          break;
        check_write = 1;
      }
    }
    response_count++;
    if (DAP_TransferAbort)
      break;
  }

  if (response_value == DAP_TRANSFER_OK)
  {
    if (post_read)
    {
      // Read previous data
      retry = DAP_Data.transfer.retry_count;
      do
      {
        response_value = SWD_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, &data);
      } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
      if (response_value != DAP_TRANSFER_OK) goto end;
      // Store previous data
      *response++ = (byte) data;
      *response++ = (byte)(data >>  8);
      *response++ = (byte)(data >> 16);
      *response++ = (byte)(data >> 24);
    }
    else if (check_write)
    {
      // Check last write
      retry = DAP_Data.transfer.retry_count;
      do
      {
        response_value = SWD_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, NULL);
      } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
    }
  }

end:
  *(response_head + 0) = (byte)response_count;
  *(response_head + 1) = (byte)response_value;

  return (response - response_head);
}

//处理SW转移块指令
static unsigned long DAP_SWD_TransferBlock(byte *request, byte *response)
{
  unsigned long  request_count;
  unsigned long  request_value;
  unsigned long  response_count;
  unsigned long  response_value;
  byte  *response_head;
  unsigned long  retry;
  unsigned long  data;
  
  response_count = 0;
  response_value = 0;
  response_head  = response;
  response      += 3;

  DAP_TransferAbort = 0;

  request++;            // Ignore DAP index

  request_count = *request | (*(request+1) << 8);
  request += 2;
  if (request_count == 0) goto end;

  request_value = *request++;
  if (request_value & DAP_TRANSFER_RnW)
  {
    // Read register block
    if (request_value & DAP_TRANSFER_APnDP)
    {
      // Post AP read
      retry = DAP_Data.transfer.retry_count;
      do
      {
        response_value = SWD_Transfer(request_value, NULL);
      } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
      if (response_value != DAP_TRANSFER_OK) goto end;
    }
    while (request_count--)
    {
      // Read DP/AP register
      if ((request_count == 0) && (request_value & DAP_TRANSFER_APnDP))
      {
        // Last AP read
        request_value = DP_RDBUFF | DAP_TRANSFER_RnW;
      }
      retry = DAP_Data.transfer.retry_count;
      do
      {
        response_value = SWD_Transfer(request_value, &data);
      } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
      if (response_value != DAP_TRANSFER_OK) goto end;
      // Store data
      *response++ = (byte) data;
      *response++ = (byte)(data >>  8);
      *response++ = (byte)(data >> 16);
      *response++ = (byte)(data >> 24);
      response_count++;
    }
  }
  else
  {
    // Write register block
    while (request_count--)
    {
      // Load data
      data =  (*(request+0) <<  0)|(*(request+1) <<  8)|(*(request+2) << 16)|(*(request+3) << 24);
      request += 4;
      // Write DP/AP register
      retry = DAP_Data.transfer.retry_count;
      do
      {
        response_value = SWD_Transfer(request_value, &data);
      } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
      if (response_value != DAP_TRANSFER_OK) goto end;
      response_count++;
    }
    // Check last write
    retry = DAP_Data.transfer.retry_count;
    do
    {
      response_value = SWD_Transfer(DP_RDBUFF | DAP_TRANSFER_RnW, NULL);
    } while ((response_value == DAP_TRANSFER_WAIT) && retry-- && !DAP_TransferAbort);
  }

end:
  *(response_head+0) = (byte)(response_count >> 0);
  *(response_head+1) = (byte)(response_count >> 8);
  *(response_head+2) = (byte) response_value;

  return (response - response_head);
}

//处理DAP主机指令
unsigned long DAP_ProcessVendorCommand(byte *request, byte *response)
{
  *response = ID_DAP_Invalid;
  return (1);
}

//处理DAP指令
unsigned long DAP_ProcessCommand(byte *request, byte *response)
{
  unsigned long num;

  if ((*request >= ID_DAP_Vendor0) && (*request <= ID_DAP_Vendor31))
  {
    return DAP_ProcessVendorCommand(request, response);
  }

  *response++ = *request;

  switch (*request++)
  {
    case ID_DAP_Info:
      num = DAP_Info(*request, response + 1);
      *response = num;
      return (2 + num);
    case ID_DAP_LED:
      num = DAP_LED(request, response);
      break;
    case ID_DAP_Connect:
      num = DAP_Connect(request, response);
      break;
    case ID_DAP_Disconnect:
      num = DAP_Disconnect(response);
      break;
    case ID_DAP_Delay:
      num = DAP_Delay(request, response);
      break;
    case ID_DAP_ResetTarget:
      num = DAP_ResetTarget(response);
      break;
    case ID_DAP_SWJ_Pins:
      num = DAP_SWJ_Pins(request, response);
      break;
    case ID_DAP_SWJ_Clock:
      num = DAP_SWJ_Clock(request, response);
      break;
    case ID_DAP_SWJ_Sequence:
      num = DAP_SWJ_Sequence(request, response);
      break;
    case ID_DAP_SWD_Configure:
      num = DAP_SWD_Configure(request, response);
      break;
    case ID_DAP_JTAG_Sequence:
    case ID_DAP_JTAG_Configure:
    case ID_DAP_JTAG_IDCODE:
      *response = DAP_ERROR;
      return (2);
    case ID_DAP_TransferConfigure:
      num = DAP_TransferConfigure(request, response);
      break;
    case ID_DAP_Transfer:
      switch (DAP_Data.debug_port)
      {
        case DAP_PORT_SWD:
          num = DAP_SWD_Transfer (request, response);
          break;
        default:
          *(response+0) = 0;    // Response count
          *(response+1) = 0;    // Response value
          num = 2;
      }
      break;
    case ID_DAP_TransferBlock:
      switch (DAP_Data.debug_port)
      {
        case DAP_PORT_SWD:
          num = DAP_SWD_TransferBlock (request, response);
          break;
        default:
          *(response+0) = 0;    // Response count [7:0]
          *(response+1) = 0;    // Response count[15:8]
          *(response+2) = 0;    // Response value
          num = 3;
      }
      break;
    case ID_DAP_WriteABORT:
      switch (DAP_Data.debug_port)
      {
        case DAP_PORT_SWD:
          num = DAP_SWD_Abort (request, response);
          break;
        default:
          *response = DAP_ERROR;
          return (2);
      }
      break;
    default:
      *(response-1) = ID_DAP_Invalid;
      return (1);
  }
  return (1 + num);
}

//设置DAP
void DAP_Setup(void)
{
  // Default settings (only non-zero values)
  //  DAP_Data.debug_port  = 0;
  //DAP_Data.fast_clock  = 0;
  DAP_Data.clock_delay = CLOCK_DELAY(DAP_DEFAULT_SWJ_CLOCK);
  //  DAP_Data.transfer.idle_cycles = 0;
  DAP_Data.transfer.retry_count = 100;
  //  DAP_Data.transfer.match_retry = 0;
  //  DAP_Data.transfer.match_mask  = 0x000000;
  DAP_Data.swd_conf.turnaround  = 1;
  //  DAP_Data.swd_conf.data_phase  = 0;
  DAP_SETUP();  // Device specific setup
}

//产生SW序列
void SWJ_Sequence (unsigned long count, byte *data)
{
  byte val;
  byte n = 0;

  while (count != 0)
  {
    count--;
    if (n == 0)
    {
      val = *data++;
      n = 8;
    }
    if (val & 1)
    {
      PIN_SWDIO_TMS_SET();
    }
    else
    {
      PIN_SWDIO_TMS_CLR();
    }
    SW_CLOCK_CYCLE();
    val >>= 1;
    n--;
  }
}

//SW转移数据        
byte SWD_TransferSlow(byte request, unsigned long *data) \
{                               \
  byte ack;                        \
  byte sbit;                        \
  unsigned long val;                       \
  byte parity;                       \
  byte n;                          \
  if(DAP_Data.fast_clock)
  {
    /* Packet Request */                    \
    parity = 0;                         \
    SWF_WRITE_BIT(1);    /* Start Bit */           \
                                  \
    sbit = request >> 0;                     \
    SWF_WRITE_BIT(sbit);    /* APnDP Bit */           \
    parity += sbit;                        \
                                  \
    sbit = request >> 1;                     \
    SWF_WRITE_BIT(sbit);    /* RnW Bit */           \
    parity += sbit;                        \
                                  \
    sbit = request >> 2;                     \
    SWF_WRITE_BIT(sbit);    /* A2 Bit */            \
    parity += sbit;                        \
                                  \
    sbit = request >> 3;                     \
    SWF_WRITE_BIT(sbit);    /* A3 Bit */            \
    parity += sbit;                        \
                                  \
    SWF_WRITE_BIT(parity); /* Parity Bit */          \
    SWF_WRITE_BIT(0);    /* Stop Bit */            \
    SWF_WRITE_BIT(1);    /* Park Bit */            \
                                  \
    /* Turnaround */                      \
    PIN_SWDIO_OUT_DISABLE();                  \
    for (n = DAP_Data.swd_conf.turnaround; n != 0; n--)     \
    {                             \
      SWF_CLOCK_CYCLE();                   \
    }                             \
                                  \
    /* Acknowledge response */                  \
    SWF_READ_BIT(sbit);                     \
    ack  = sbit << 0;                      \
                                  \
    SWF_READ_BIT(sbit);                     \
    ack |= sbit << 1;                      \
                                  \
    SWF_READ_BIT(sbit);                     \
    ack |= sbit << 2;                      \
                                  \
    if (ack == DAP_TRANSFER_OK)                 \
    { /* OK response */                   \
      /* Data transfer */                   \
      if (request & DAP_TRANSFER_RnW)             \
      { /* Read data */                   \
        val = 0;                      \
        parity = 0;                     \
        for (n = 32; n; n--)                \
        {                         \
          SWF_READ_BIT(sbit); /* Read RDATA[0:31] */    \
          parity += sbit;                  \
          val >>= 1;                    \
          val  |= sbit << 31;                \
        }                         \
        SWF_READ_BIT(sbit);   /* Read Parity */     \
        if ((parity ^ sbit) & 1)               \
        {                         \
          ack = DAP_TRANSFER_ERROR;           \
        }                         \
        if (data) *data = val;                \
        /* Turnaround */                  \
        for (n = DAP_Data.swd_conf.turnaround; n != 0; n--) \
        {                         \
          SWF_CLOCK_CYCLE();               \
        }                         \
                                  \
        PIN_SWDIO_OUT_ENABLE();               \
      }                           \
      else                          \
      {                           \
        /* Turnaround */                  \
        for (n = DAP_Data.swd_conf.turnaround; n != 0; n--) \
        {                         \
          SWF_CLOCK_CYCLE();               \
        }                         \
                                  \
        PIN_SWDIO_OUT_ENABLE();               \
        /* Write data */                  \
        val = *data;                    \
        parity = 0;                     \
        for (n = 32; n; n--) {                \
          SWF_WRITE_BIT(val);  /* Write WDATA[0:31] */   \
          parity += val;                  \
          val >>= 1;                    \
        }                         \
        SWF_WRITE_BIT(parity); /* Write Parity Bit */    \
      }                           \
      /* Idle cycles */                   \
      n = DAP_Data.transfer.idle_cycles;            \
      if (n != 0)                       \
      {                           \
        PIN_SWDIO_OUT(0);                 \
        for (; n != 0; n--)                 \
        {                         \
          SWF_CLOCK_CYCLE();               \
        }                         \
      }                           \
      PIN_SWDIO_OUT(1);                   \
      return (ack);                     \
    }                             \
                                  \
    if (ack == DAP_TRANSFER_WAIT || ack == DAP_TRANSFER_FAULT)  \
    {                                     \
      /* WAIT or FAULT response */                      \
      if (DAP_Data.swd_conf.data_phase && (request & DAP_TRANSFER_RnW) != 0)  \
      {                                   \
        for (n = 32+1; n; n--)                        \
        {                                 \
          SWF_CLOCK_CYCLE(); /* Dummy Read RDATA[0:31] + Parity */   \
        }                                 \
      }                                   \
      /* Turnaround */                            \
      for (n = DAP_Data.swd_conf.turnaround; n != 0; n--)           \
      {                                   \
        SWF_CLOCK_CYCLE();                         \
      }                                   \
                                          \
      PIN_SWDIO_OUT_ENABLE();                         \
      if (DAP_Data.swd_conf.data_phase && (request & DAP_TRANSFER_RnW) == 0)  \
      {                                   \
        PIN_SWDIO_OUT(0);                         \
        for (n = 32 + 1; n != 0; n--)                   \
        {                                 \
          SWF_CLOCK_CYCLE(); /* Dummy Write WDATA[0:31] + Parity */    \
        }                                 \
      }                                   \
      PIN_SWDIO_OUT(1);                           \
      return (ack);                             \
    }                                     \
                                          \
    /* Protocol error */                            \
    for (n = DAP_Data.swd_conf.turnaround + 32 + 1; n != 0; n--)        \
    {                                     \
      SWF_CLOCK_CYCLE(); /* Back off data phase */             \
    }                                     \
                                          \
    PIN_SWDIO_OUT(1);                             \
  }else
  {
      /* Packet Request */                    \
    parity = 0;                         \
    SW_WRITE_BIT(1);    /* Start Bit */           \
                                  \
    sbit = request >> 0;                     \
    SW_WRITE_BIT(sbit);    /* APnDP Bit */           \
    parity += sbit;                        \
                                  \
    sbit = request >> 1;                     \
    SW_WRITE_BIT(sbit);    /* RnW Bit */           \
    parity += sbit;                        \
                                  \
    sbit = request >> 2;                     \
    SW_WRITE_BIT(sbit);    /* A2 Bit */            \
    parity += sbit;                        \
                                  \
    sbit = request >> 3;                     \
    SW_WRITE_BIT(sbit);    /* A3 Bit */            \
    parity += sbit;                        \
                                  \
    SW_WRITE_BIT(parity); /* Parity Bit */          \
    SW_WRITE_BIT(0);    /* Stop Bit */            \
    SW_WRITE_BIT(1);    /* Park Bit */            \
                                  \
    /* Turnaround */                      \
    PIN_SWDIO_OUT_DISABLE();                  \
    for (n = DAP_Data.swd_conf.turnaround; n != 0; n--)     \
    {                             \
      SW_CLOCK_CYCLE();                   \
    }                             \
                                  \
    /* Acknowledge response */                  \
    SW_READ_BIT(sbit);                     \
    ack  = sbit << 0;                      \
                                  \
    SW_READ_BIT(sbit);                     \
    ack |= sbit << 1;                      \
                                  \
    SW_READ_BIT(sbit);                     \
    ack |= sbit << 2;                      \
                                  \
    if (ack == DAP_TRANSFER_OK)                 \
    { /* OK response */                   \
      /* Data transfer */                   \
      if (request & DAP_TRANSFER_RnW)             \
      { /* Read data */                   \
        val = 0;                      \
        parity = 0;                     \
        for (n = 32; n; n--)                \
        {                         \
          SW_READ_BIT(sbit); /* Read RDATA[0:31] */    \
          parity += sbit;                  \
          val >>= 1;                    \
          val  |= sbit << 31;                \
        }                         \
        SW_READ_BIT(sbit);   /* Read Parity */     \
        if ((parity ^ sbit) & 1)               \
        {                         \
          ack = DAP_TRANSFER_ERROR;           \
        }                         \
        if (data) *data = val;                \
        /* Turnaround */                  \
        for (n = DAP_Data.swd_conf.turnaround; n != 0; n--) \
        {                         \
          SW_CLOCK_CYCLE();               \
        }                         \
                                  \
        PIN_SWDIO_OUT_ENABLE();               \
      }                           \
      else                          \
      {                           \
        /* Turnaround */                  \
        for (n = DAP_Data.swd_conf.turnaround; n != 0; n--) \
        {                         \
          SW_CLOCK_CYCLE();               \
        }                         \
                                  \
        PIN_SWDIO_OUT_ENABLE();               \
        /* Write data */                  \
        val = *data;                    \
        parity = 0;                     \
        for (n = 32; n; n--) {                \
          SW_WRITE_BIT(val);  /* Write WDATA[0:31] */   \
          parity += val;                  \
          val >>= 1;                    \
        }                         \
        SW_WRITE_BIT(parity); /* Write Parity Bit */    \
      }                           \
      /* Idle cycles */                   \
      n = DAP_Data.transfer.idle_cycles;            \
      if (n != 0)                       \
      {                           \
        PIN_SWDIO_OUT(0);                 \
        for (; n != 0; n--)                 \
        {                         \
          SW_CLOCK_CYCLE();               \
        }                         \
      }                           \
      PIN_SWDIO_OUT(1);                   \
      return (ack);                     \
    }                             \
                                  \
    if (ack == DAP_TRANSFER_WAIT || ack == DAP_TRANSFER_FAULT)  \
    {                                     \
      /* WAIT or FAULT response */                      \
      if (DAP_Data.swd_conf.data_phase && (request & DAP_TRANSFER_RnW) != 0)  \
      {                                   \
        for (n = 32+1; n; n--)                        \
        {                                 \
          SW_CLOCK_CYCLE(); /* Dummy Read RDATA[0:31] + Parity */   \
        }                                 \
      }                                   \
      /* Turnaround */                            \
      for (n = DAP_Data.swd_conf.turnaround; n != 0; n--)           \
      {                                   \
        SW_CLOCK_CYCLE();                         \
      }                                   \
                                          \
      PIN_SWDIO_OUT_ENABLE();                         \
      if (DAP_Data.swd_conf.data_phase && (request & DAP_TRANSFER_RnW) == 0)  \
      {                                   \
        PIN_SWDIO_OUT(0);                         \
        for (n = 32 + 1; n != 0; n--)                   \
        {                                 \
          SW_CLOCK_CYCLE(); /* Dummy Write WDATA[0:31] + Parity */    \
        }                                 \
      }                                   \
      PIN_SWDIO_OUT(1);                           \
      return (ack);                             \
    }                                     \
                                          \
    /* Protocol error */                            \
    for (n = DAP_Data.swd_conf.turnaround + 32 + 1; n != 0; n--)        \
    {                                     \
      SW_CLOCK_CYCLE(); /* Back off data phase */             \
    }                                     \
                                          \
    PIN_SWDIO_OUT(1);                             \
  }
  return (ack);                               \
}

//SW转移数据
byte SWD_Transfer(byte request, unsigned long *data)
{
  return SWD_TransferSlow(request, data);
}

//DAP执行指令
unsigned long DAP_ExecuteCommand(byte *request, byte *response) 
{
  unsigned long cnt, num, n;

  if (*request == 0x7FU) 
  {
    *response++ = *request++;
    cnt = *request++;
    *response++ = (byte)cnt;
    num = (2U << 16) | 2U;
    while (cnt--) 
    {
      n = DAP_ProcessCommand(request, response);
      num += n;
      request  += (word)(n >> 16);
      response += (word) n;
    }
    return (num);
  }

  return DAP_ProcessCommand(request, response);
}

#endif /* __DAP_H__ */
